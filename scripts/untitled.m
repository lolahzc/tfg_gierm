%% Initialize ROS2-Matlab connection
% RMW configuration — choose the SAME middleware used by the system nodes
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');   

if isempty(getenv('ROS_DOMAIN_ID'))
    setenv('ROS_DOMAIN_ID', '16');
end

% Read the mission id from the config file
config_file_path = '../config/conf.yaml';
config_file = fopen(config_file_path, 'r');
scenario_id = fscanf(config_file, 'mission_id: %s');
fclose(config_file);

% Load scenario's data
[Agent, Task] = scenario(scenario_id);
userData.Agent = Agent;
userData.Task = Task;

%% Node
if ~exist('node', 'var')
    node = ros2node('/matlab_ros_connector');
end

%% Subscribers
mission_over_sub = ros2subscriber(node, '/mission_over', 'mission_planner/MissionOver');

%% Action servers
planning_server = ros2actionserver(node, '/heuristic_planning', ...
    'mission_planner/HeuristicPlanning', ...
    ExecuteGoalFcn=@(server, goalStruct, defaultResult) heuristicPlanningCallback(userData, server, goalStruct, defaultResult));

disp('Heuristic planning action server is ready');

% Initialize loop variable
mission_over = false;

while not(mission_over)
    try
        [msg, ~, ~] = receive(mission_over_sub, 0.5);
        if msg.value == true
            mission_over = true;
        end
    catch
    end
    pause(0.5);
end

clear planning_server mission_over_sub node;

%% Callback
function [resultMsg, success] = heuristicPlanningCallback(userData, server, goalStruct, defaultResult)
    % Usamos el defaultResult si existe, si no, instanciamos uno nuevo
    if ~isempty(defaultResult)
        resultMsg = defaultResult;
    else
        resultMsg = ros2message('mission_planner/HeuristicPlanningResult');
    end
    success = false;
    % Desempaquetar Goal
    if isstruct(goalStruct) && isfield(goalStruct, 'Goal')
        goalMsg = goalStruct.Goal;
    elseif isobject(goalStruct) && isprop(goalStruct, 'Goal')
        goalMsg = goalStruct.Goal;
    else
        goalMsg = goalStruct; 
    end

    if isstruct(goalMsg) && isfield(goalMsg, 'goal')
        payload = goalMsg.goal;
    elseif isobject(goalMsg) && isprop(goalMsg, 'goal')
        payload = goalMsg.goal;
    else
        payload = goalMsg;
    end

    disp('--------------------------------------');
    disp('Received a new planning request');

    % 1. DETECCIÓN DE PROPIEDADES en el PAYLOAD REAL
    if isstruct(payload)
        props = fieldnames(payload);
    else
        props = properties(payload);
    end
    
    if ismember('available_agents', props)
        f_agents = 'available_agents';
        f_tasks = 'remaining_tasks';
    elseif ismember('AvailableAgents', props)
        f_agents = 'AvailableAgents';
        f_tasks = 'RemainingTasks';
    elseif ismember('Available_agents', props)
        f_agents = 'Available_agents';
        f_tasks = 'Remaining_tasks';
    else
        disp('ERROR CRÍTICO: No encuentro los arrays en el payload.');
        return; 
    end

    % 2. Extraemos los robots disponibles
    robot_counter = 1;
    available_agents = [];
    requested_agents = string(payload.(f_agents));
    
    for robot = 1:size(userData.Agent, 2)
        if ismember(userData.Agent(robot).name, requested_agents)
            if isempty(available_agents)
                available_agents = userData.Agent(robot);
            else
                available_agents(robot_counter) = userData.Agent(robot);
            end
            robot_counter = robot_counter + 1;
        end
    end

    % 3. Extraemos las tareas pendientes
    pending_tasks = userData.Task(1);
    task_counter = 2;
    requested_tasks = string(payload.(f_tasks));
    
    for task = 2:1:size(userData.Task, 2)
        if ismember(userData.Task(task).name, requested_tasks)
            pending_tasks(task_counter) = userData.Task(task);
            task_counter = task_counter + 1;
        end
    end

    % --- FEEDBACK ---
    fb = ros2message('mission_planner/HeuristicPlanningFeedback');
    
    if isstruct(fb)
        fb_props = fieldnames(fb);
    else
        fb_props = properties(fb);
    end
    
    if ismember('status', fb_props)
        fb.status = 'Executing heuristic planner...';
    elseif ismember('Status', fb_props)
        fb.Status = 'Executing heuristic planner...';
    elseif ismember('feedback', fb_props)
        fb.feedback.status = 'Executing heuristic planner...';
    elseif ismember('Feedback', fb_props)
        fb.Feedback.Status = 'Executing heuristic planner...';
    end
    
    % MANDAMOS EL FEEDBACK AL SERVIDOR (COMO TENÍA QUE SER)
    sendFeedback(server, fb);

    % --- LLAMADA AL ASIGNADOR DE TAREAS CORREGIDA (CON EL 1 DE SEMILLA) ---
    try
        [Agent, Task, ~, ~] = heuristicTaskAllocator(available_agents, pending_tasks, 4, 9, 1);
        is_success = true;
    catch ME
        disp('Error executing heuristicTaskAllocator:');
        disp(ME.message);
        is_success = false;
    end

    % --- ARREGLO DEL RESULTADO ---
    if isstruct(resultMsg)
        out_props = fieldnames(resultMsg);
    else
        out_props = properties(resultMsg);
    end
    
    has_result = false;
    if ismember('result', out_props)
        res_obj = resultMsg.result;
        has_result = true;
        f_res = 'result';
    elseif ismember('Result', out_props)
        res_obj = resultMsg.Result;
        has_result = true;
        f_res = 'Result';
    else
        res_obj = resultMsg;
    end

    if isstruct(res_obj)
        res_props = fieldnames(res_obj);
    else
        res_props = properties(res_obj);
    end
    
    if ismember('planning_result', res_props)
        f_success = 'success';
        f_plan_res = 'planning_result';
        f_agent_id = 'agent_id';
        f_queue = 'queue';
        f_task_id = 'id';
    else
        f_success = 'Success';
        f_plan_res = 'PlanningResult';
        f_agent_id = 'AgentId';
        f_queue = 'Queue';
        f_task_id = 'Id';
    end

    res_obj.(f_success) = is_success;
    success = is_success; 

    if is_success == true
        for robot = 1:size(Agent, 2)
            tq = ros2message('mission_planner/TaskQueue');
            tq.(f_agent_id) = Agent(robot).name;
            
            task_msgs = [];
            for task = 2:size(Agent(robot).queue, 2)
                t = ros2message('mission_planner/Task');
                t.(f_task_id) = Task(Agent(robot).queue(task)).name;
                if isempty(task_msgs)
                    task_msgs = t;
                else
                    task_msgs(end+1) = t; %#ok<AGROW>
                end
            end
            
            if ~isempty(task_msgs)
                tq.(f_queue) = task_msgs;
            end
            
            if robot == 1
                res_obj.(f_plan_res) = tq;
            else
                res_obj.(f_plan_res)(robot) = tq;
            end
        end
    end

    if has_result
        resultMsg.(f_res) = res_obj;
    else
        resultMsg = res_obj;
    end

    % Enviamos el feedback final
    if ismember('status', fb_props)
        fb.status = 'Sending back the planning result...';
    elseif ismember('Status', fb_props)
        fb.Status = 'Sending back the planning result...';
    elseif ismember('feedback', fb_props)
        fb.feedback.status = 'Sending back the planning result...';
    elseif ismember('Feedback', fb_props)
        fb.Feedback.Status = 'Sending back the planning result...';
    end
    
    sendFeedback(server, fb);
    
    disp('Planning finished and result sent back.');
    disp('--------------------------------------');
end