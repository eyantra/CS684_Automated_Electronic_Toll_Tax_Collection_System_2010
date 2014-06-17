%%-----------------------------------------------------------------------------------
% Code_TollPlaza.m:- Main Program for ZIGBEE Communication 
% Between TollPlaza and Vehicle
%
%------------------------------------------------------------------------------------
% Author:
% 		Puskar Kothavade
%		Ashish Pardhi
% 		Mugdha Nazare
%-------------------------------------------------------------------------------------

%   Copyright (c) 2010. ERTS Lab IIT Bombay
%   All rights reserved.

%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions are met:

%   * Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%
%   * Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in
%     the documentation and/or other materials provided with the
%     distribution.

%   * Neither the name of the copyright holders nor the names of
%     contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.

%   * Source code can be used for academic purpose. 
%	 For commercial use permission form the author needs to be taken.
%--------------------------------------------------------------------------------------

%%%%%%%%%%%% Toll Plaza (Windows Based Computer) %%%%%%%%%%%%%
%% Start %%

clc;                                % Clear Screen.
warning off all;                    % Do Not Show Any Warnings.
c = '0'                             % Initialise Counter.
while(1)                            % Run Following Program Continuously.
    
%% Communication Module Through ZigBee %%

s = serial('COM13');                % Define COM Port Object To Communicate To Vehicle.
fopen(s);                           % Open COM port. 
id = '';                            % Initialise id Variable.
w = size(id);                       % Find Out Whether id Variable is empty or Non-empty
while ( w(1) == 0)
fprintf(s,'%c','I');                % Send character 'I' to vehicle to get the Vehicle ID.
id = fscanf(s)                      % Read vehicle ID.
w = size(id);
end
fprintf(s,'%c','Z');                % Toll plaza starts communicating with vehicle
Start = 1
speed1 = '';                        % Procedure To Calculate Speed Of The Vehicle.
m = size(speed1);                   
while( m(1)== 0)
fprintf(s,'%c','X');
speed1 = fscanf(s);
m = size(speed1);
end

speed2 = '';
n = size(speed2);
while( n(1)== 0)
fprintf(s,'%c','Y');
speed2 = fscanf(s);
n = size(speed2);
end

speed1 = speed1 - 48 + 48               % Speed1 Is Number Left Of Decimal Point.
speed2 = speed2 - 48 + 48               % Speed2 Is Number Right Of Decimal Point.
TotalSpeed = speed1 + (0.01 * speed2)   % TotalSpeed = Speed1.Speed2

Port2 = serial('COM12');                % Define COM Port Object To Communicate To Linux Based Computer.
fopen(Port2);                           % Open COM Port. 

%% Database Access %%
id = id - 48;
if (id > 0 && id < 5)
    z = xlsread('AETC_Record.xls');         % Read Excel Sheet Having User Database.
    if  (z(id,2) >= 100)            % Minimum balance has to be 100 rupees.
        z(id,2) = z(id,2) - 100;    % Deduct 100 Rupees From Account.
        z(id,3) = 0;                % For Successful Transaction Make Status = 0.
        id = id + 48;
        fprintf(Port2,'%c',id);     % Send Vehicle's ID and No violation Status To Linux Based PC.
        fprintf(Port2,'%c','S');
    else
        z(id,2) = 0;                % If Balance Is Less Than 100 Rupees Then Deduct Remaining Balance.
        z(id,3) = z(id,3)+1;        % Make status = 1 If Violation Is Detected.
        violate = 1;                % Set violation Variable Equals To One.
        id = id + 48;
        fprintf(Port2,'%c',id);     % Send Vehicle's ID and Violation Status To Linux Based PC.
        fprintf(Port2,'%c','F');
    end ; 
    id = id - 48;
    z(id,4) = TotalSpeed;
    clk = clock;
    z(id,5) = clk(3);
    z(id,6) = clk(2);
    z(id,7) = clk(1);
    z(id,8) = clk(4);
    z(id,9) = clk(5);
    z(id,10) = clk(6);
    xlswrite('AETC_Record.xls', z);         % Update The Database.
end;
fclose(Port2);                      % Close Port.

%% Image Capture %% 

v = ''                              % Initialise Variable v
k = size(v);
while ( k(1) == 0)
fprintf(s,'%c','V');                % Send Character 'V' To Vehicle To Get The Flag Value.
v = fscanf(s)                       % If Flag 'v' is one, Tt Means That Vehicle Has Crossed Third Black Patch.
k = size(v);
end
c = c + 1;
a = [c '.' 'j' 'p' 'g'];
A = ['A' c];
vid=videoinput('winvideo',2);       % Start USB Camera To Capture Image.
triggerconfig(vid,'manual');
config = triggerinfo(vid);
set(vid,'FramesPerTrigger',1);
set(vid,'TriggerRepeat', Inf);
start(vid); 
for i=1:20
    trigger(vid);
    im= getdata(vid,1);
end
stop(vid),delete(vid),clear vid;
cd LicensePlatePhotos ;
imwrite(im,a);
fprintf(s,'%c','D');

%% Image Transformation %%

mm = imread(a);                     % Convert Image From .jpg To .pbm Format.
mm=imcomplement(mm);                % Complement The Image.
cd ..;                              % Change Directory. 
imwrite(mm,'C:\Users\pushkarbk\Documents\MATLAB\license.pbm');      

%% GOCR %%

[status, result] = dos('gocr048.exe -m 4 C:\Users\pushkarbk\Documents\MATLAB\license.pbm');  % Apply Optical Character Recognition (OCR) Algorithm On Captured Image Of License Plate.
xlswrite('AETC_LicNum.xls', result, 'AETC_LicNum',A);

%% End %%
fclose(s);                          % Close Port.
end                                 % End Of While(1) Loop.


%%--------------------------------------
% End of Program.
%%--------------------------------------
