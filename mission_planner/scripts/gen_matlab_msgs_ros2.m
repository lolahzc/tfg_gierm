% Este script compila las acciones y mensajes de ROS 2 para MATLAB
% Ejecútalo desde dentro de la carpeta src de tu workspace de ROS 2

disp('Generando mensajes personalizados de ROS 2 para MATLAB...');
folderPath = pwd; % Debe apuntar a la ruta de tu paquete (donde esté package.xml)

try
    ros2genmsg(folderPath);
    disp('¡Mensajes compilados! Añadiendo al path...');
    
    % Añadir la ruta generada al Path de MATLAB
    addpath(fullfile(folderPath, 'matlab_msg_gen_ros2', 'glnxa64', 'install', 'm'));
    savepath;
    
    clear classes
    rehash toolboxcache
    disp('Sistema listo para usar ROS 2 en MATLAB.');
catch EM
    error('Error generando mensajes de ROS 2: %s', EM.message);
end