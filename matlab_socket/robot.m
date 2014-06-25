%%
import java.net.Socket
import java.io.*

%%

% connect to robot
socket = Socket('192.168.7.2', 2002);

% get streams tied to socket
output_stream   = socket.getOutputStream;
d_output_stream = DataOutputStream(output_stream);

% input_stream   = socket.getInputStream;
% d_input_stream = DataInputStream(input_stream);

%%
d_output_stream.write([97 0 0 1]);
d_output_stream.flush;

%%
% pause(0.01);
% bytes_available = input_stream.available;
% message = d_input_stream.readLine;
% disp(message);

socket.close;