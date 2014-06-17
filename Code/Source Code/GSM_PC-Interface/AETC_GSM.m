%%-----------------------------------------------------------------------------------
% Code_GSM.m:- Main Program for GSM Communication 
% Between TollPlaza and Vehicle Owner
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

% Electronic Toll Tax Collection System %

clc;
warning off all;
s = serial('/dev/ttyUSB0');                                                % Open Serial Port.         
fopen(s);
   

while(1)
    
    id = fscanf(s)                                                         % Read Serial Port
    w = size(id);                                                          % Find Out Whether Variable id Empty Or Non-empty
 
    if (w(1) > 0)                                                          % If Variable id Is Non-empty Then Only Check Further Code. 

        if (id(1) == '1')	
            if(id(2) == 'S')           
                unix('./id1s');                                            % Execute Precompiled 'idls' File. 
            end       
            if(id(2) == 'F')
                unix('./id1f');                                            % Execute Precompiled 'idlf' File. 
            end 	   
        end

        if (id(1) == '2')
            if(id(2) == 'S')           
                unix('./id2s');   
            end       
            if(id(2) == 'F')
                unix('./id2f');
            end 	   
        end

        if (id(1) == '3')	
            if(id(2) == 'S')           
                unix('./id3s');   
            end       
            if(id(2) == 'F')
                unix('./id3f');
            end 	   
        end    

        if (id(1) == '4')	
            if(id(2) == 'S')           
                unix('./id4s');   
            end       
            if(id(2) == 'F')
                unix('./id4f');
            end 	   
        end

    end

 end

fclose(s);                                                                 % Close Serial Port. 
