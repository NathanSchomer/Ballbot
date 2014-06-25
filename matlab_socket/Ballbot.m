classdef Ballbot < handle
    %A class for communicating with the Ballbot
    %   Detailed explanation goes here
    
    properties
        address;        
        port;            %the COM port of the snake's serial interface
    end
    
    properties (GetAccess = 'private', SetAccess = 'private')

    end
    
    properties (Constant, GetAccess = 'private')
        MID_echo        = 0;
        MID_getStatus   = 1;
        MID_getIMU      = 2;
        MID_setGains    = 3;
        MID_trimIMU     = 4;
    end
   
    methods
        
        function obj = Ballbot(address, port)    
            obj.address = address;
            obj.port = port;
        end
        
        function writeInt8(obj, MID, data)
            data_as_uint8 = int8_to_uint8(data);
            message = buildMessage(MID, data_as_uint8);
            writeRaw(obj, message);
        end
        
        function writeInt16(obj, MID, data)
            data_as_uint8 = int16_to_uint8(data);
            message = buildMessage(MID, data_as_uint8);
            writeRaw(obj, message);
        end
        
        function writeSingle(obj, MID, data)
            data_as_uint8 = single_to_uint8(data);
            message = buildMessage(MID, data_as_uint8);
            writeRaw(obj, message);
        end
        
        function m = writeRaw(obj, raw)
            import java.net.Socket
            import java.net.InetSocketAddress
            import java.io.*
            
            % open socket and get IO streams
            sock = Socket(obj.address, obj.port);
            
            out_stream = BufferedOutputStream(sock.getOutputStream);
            in_stream = BufferedInputStream(sock.getInputStream);

            out_stream.write(raw);
            out_stream.flush;
            
%             pause(0.1)
            % reading header bytes and casting into uint16's
            header = zeros(1, 4, 'uint8');
            for i = 1:4
                header(i) = in_stream.read;
            end

            m.MID = typecast(uint8(header(1:2)), 'uint16');
            m.length = typecast(uint8(header(3:4)), 'uint16');
            
            % reading raw data bytes
            data = zeros(1, m.length, 'uint8');
            for i = 1:m.length
                data(i) = in_stream.read;
            end
            
            m.data = data;
            sock.close;
        end
        
        function m = writeMID(obj, MID, message_uint8)
            raw = [MID typecast(int16(length(message_uint8)), 'uint8') message_uint8];
        end
        
        function m = writeGain(obj, gain)
            if length(gain) == 6
                obj.writeRaw([3 0 24 0 typecast(single(gain), 'uint8')])
            else
                disp('Command requires 1-by-6 vector');
            end
        end
        
        function m = trimIMU(obj, gain)
            if length(gain) == 2
                obj.writeRaw([4 0 8 0 typecast(single(gain), 'uint8')])
            else
                disp('Command requires 1-by-3 vector');
            end
        end
        
    end
    
end

% buildMessage() builds properly formatted message strings
% input args
%%% address => address of segment
%%% MID => the message indentifier
%%% data => the data payload for this message
% returne arg
%%% message => the formatted message char string
%|  length   |   MID   |   address  | data  |
%%% length => length of DATA section
%%% MID => instruction identifier
%%% address => address of destination node
%%% data => optional data payload

function message = buildMessage(MID, data)

    message = [MID typecast(int16(length(data)), 'uint8') cast(data, 'uint8')];

end

% int8tochars() turns a signed int8 in range [-128 128] into a char
% It can take vector inputs as well.
function chars = int8_to_uint8(number)
    chars = (typecast(int8(number), 'uint8'));
end

% int16tochars() breaks a signed int16 in range into two chars
% This is how to break an unsigned 16 bit int into its upper and lower bytes
% then reassemble it:
%%% a = char(typecast(uint16(1025), 'uint8'))
%%% typecast(uint8(a), 'uint16')
function chars = int16_to_uint8(number)
    chars = (typecast(int16(number), 'uint8'));
end


function chars = single_to_uint8(number)
    chars = (typecast(single(number), 'uint8'));
end
