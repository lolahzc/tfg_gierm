%% MATLAB ROS 2 Connector - Versión Final Corregida
clear all;
clc;
clear mex;

%% 1. Inicialización y Carga de Datos
config_file_path = '../config/conf.yaml';
config_file = fopen(config_file_path, 'r');
scenario_id = fscanf(config_file, 'mission_id: %s');
fclose(config_file);

% Cargar escenario
[Agent_mat, Task_mat] = scenario(scenario_id);

% 🛠️ Sincronizar nombres con los drones de Gazebo (drone0, drone1...)
for i = 1:length(Agent_mat)
    Agent_mat(i).name = sprintf('drone%d', i-1); 
end
userData.Agent = Agent_mat;
userData.Task = Task_mat;

%% 2. Nodo ROS 2
if ~exist('node', 'var')
    node = ros2node('/matlab_ros_connector');
end

%% 3. Inyector de Misiones (Sustituye al Sequencer)
nt_client = ros2actionclient(node, '/incoming_task_action', 'mission_planner/NewTask');

disp('⏳ Esperando al Planner de C++...');
if waitForServer(nt_client, "Timeout", 15) 
    disp('🚀 Planner detectado. Inyectando misiones desde Task.mat...');
    
    for i = 2:length(Task_mat)
        m = Task_mat(i);
        goal = ros2message(nt_client);
        
        % 🛠️ CORRECCIÓN CRÍTICA: Usar char() en lugar de string()
        goal.task.id = char(m.name); 
        
        % Mapeo de tipos (ASCII uint8) basado en el nombre o propiedades
        task_name_lower = lower(char(m.name));
        
        if contains(task_name_lower, 'monitor')
            goal.task.type = uint8('M'); 
        elseif contains(task_name_lower, 'inspect') || m.Relayability == 1
            goal.task.type = uint8('I');
        elseif contains(task_name_lower, 'deliver')
            goal.task.type = uint8('D');
        else
            % Por defecto si no se detecta por nombre
            if m.Relayability == 1, goal.task.type = uint8('I');
            else, goal.task.type = uint8('M'); end
        end
        
        % Enviar la misión al Planner C++
        sendGoal(nt_client, goal);
        fprintf('   - Tarea enviada: %s (Tipo: %c)\n', goal.task.id, char(goal.task.type));
        pause(0.2); 
    end
    disp('✅ Inyección completa.');
else
    disp('⚠️ Error: No se pudo contactar con el Planner C++.');
end

%% 4. Servidor Heurístico
action_type = 'mission_planner/HeuristicPlanning'; 
planning_server = ros2actionserver(node, '/heuristic_planning', action_type, ...
    'ExecuteGoalFcn', {@heuristicPlanningCallback, userData});

mission_over_sub = ros2subscriber(node, '/mission_over', 'mission_planner/MissionOver');
disp('✨ Servidor listo y esperando peticiones de planificación...');

%% 5. Bucle Principal
mission_over = false;
while ~mission_over
    if ~isempty(mission_over_sub.LatestMessage)
        % Nota: Verifica si tu mensaje usa .value o .Value
        if mission_over_sub.LatestMessage.value == true
            mission_over = true;
            disp('🏁 Misión terminada.');
        end
    end
    pause(0.5);
end

clear nt_client planning_server mission_over_sub node;

%% 6. Callback de Planificación
function [result_msg, success] = heuristicPlanningCallback(src, goalStruct, defaultFeedbackMsg, defaultResultMsg, userData)
    disp('>>> 🟢 Petición de planificación recibida!');
    
    goal_msg = goalStruct.goal;
    result_msg = defaultResultMsg;
    
    % Extraer datos de la red ROS 2
    cxx_agents = cellstr(goal_msg.available_agents);
    cxx_tasks  = cellstr(goal_msg.remaining_tasks);
    
    if isempty(cxx_agents) || isempty(cxx_tasks)
        disp('⚠️ Petición vacía. Devolviendo plan nulo.');
        result_msg.success = true;
        success = true;
        return;
    end

    % Filtrar estructuras locales
    robot_counter = 1;
    available_agents = [];
    for i = 1:length(userData.Agent)
        if ismember(userData.Agent(i).name, cxx_agents)
            available_agents(robot_counter) = userData.Agent(i);
            robot_counter = robot_counter + 1;
        end
    end

    pending_tasks(1) = userData.Task(1); % t_R
    task_counter = 2;
    for i = 2:length(userData.Task)
        if ismember(userData.Task(i).name, cxx_tasks)
            pending_tasks(task_counter) = userData.Task(i);
            task_counter = task_counter + 1;
        end
    end

    % Ejecutar algoritmo
    try
        [Agent_res, Task_res] = heuristicTaskAllocator(available_agents, pending_tasks, 4, 9);
        result_msg.success = true;
        success = true;
        
        for r = 1:length(Agent_res)
            tq_msg = ros2message('mission_planner/TaskQueue');
            tq_msg.agent_id = char(Agent_res(r).name);
            for t_idx = 2:length(Agent_res(r).queue)
                t_msg = ros2message('mission_planner/Task');
                t_msg.id = char(Task_res(Agent_res(r).queue(t_idx)).name);
                tq_msg.queue(t_idx - 1) = t_msg;
            end
            result_msg.planning_result(r) = tq_msg;
        end
        disp('>>> ✅ Plan enviado.');
    catch ME
        disp(['❌ Error: ', ME.message]);
        result_msg.success = false;
        success = false;
    end
    
    % Feedback obligatorio
    defaultFeedbackMsg.status = char('Planning complete');
    sendFeedback(src, goalStruct, defaultFeedbackMsg);
end