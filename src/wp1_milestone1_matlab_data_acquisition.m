%% Preparing the serial communication
% https://www.mathworks.com/help/matlab/import_export/read-streaming-data-from-arduino.html
% Clear the workspace
% Select the correct port and correct Baudrate
% Set the terminator to "CR/LF" (CR = Carriage Return and LF = Line Feed)
clc; clear; close all; format compact; format shortG
s = serialport('COM3', 9600);
configureTerminator(s, "CR/LF");

% Use this code for real-time data vizualisation (plot angle vs time)
clc; close all; flush(s);

% Create a variable to store user input
global userInput;
userInput = "";
% Prepare the parameters for the animated line

%setup figure for our graph, most important thing is keyPressCallback,
%which listens for keys and sends commands to serial output
hFig = figure('KeyPressFcn', @keyPressCallback, 'Name', 'MATLAB Command Window');

%create 3 animated lines for our code
h1 = animatedline('Color','b','LineWidth',2, 'MaximumNumPoints',500); 
h2 = animatedline('Color','r','LineWidth',2, 'MaximumNumPoints',500); 
grid on;

%setup window size
screen_property = get(0,'screensize');
set(gcf, "OuterPosition", [0, screen_property(4)/2, ...
    screen_property(3)/2, screen_property(4)/2])
xlabel("Time (s)");

drawnowc

% Start the serial COM reading and animation
% Break the loop with Ctrl+C
while 1
    % write to serial
    if (~strcmp(userInput, ''))
        writeline(s, userInput);
        userInput = '';
    end
    %read from serial communication
    %do NOT add a semicolon so we can see its outputs via the command window
    string = readline(s);
    %scans the string for inputs: %f 
    %double %% means one % in the string
    if (size(string) == 0)
        continue;
    end
    disp(string)
    data = sscanf(string, "Loadcell: %f g\nMotor usage: %f%%\nCurrent: %f A\nVoltage: %f V\nPower: %f W\nTime: %f\n");
    %make sure the data is correct i.e. 6 outputs is extracted from string
    if (size(data) < 6) 
        continue;
    end
    %assign the variables
    loadcell = data(1);
    motorUsage = data(2);
    current = data(3);
    voltage = data(4);
    power = data(5);
    time = data(6);
    %set xlim to move our graph horizontally
    xlim([max(0, time/1000 - 10), time/1000 + 10]);
    %set ylim to resize our graph upward
    ylim([-5, min(250, round(loadcell/40)*40 + 40)])
    
    %add points to the corresponding lines
    addpoints(h1, time/1000, loadcell)
    addpoints(h2, time/1000, power)

    % Comment the next row to stop data write into an excel file
    %writematrix([loadcell power voltage current time/1000],'loadcell_powerconsumption.xlsx','WriteMode','append')
end

% Command Window KeyPressFcn callback functions
function keyPressCallback(src, event)
    % Use the global variable 'userInput' inside the callback function
    global userInput;
    % Get the pressed key
    key = event.Key;

    % Handle the user input
    if (strcmp(key, 's'))
        userInput = key;
    end
end