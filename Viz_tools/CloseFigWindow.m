function CloseFigWindow(~,~,msgbx)

msgbx.String  = 'SELECTION COMPLETED...CLOSING IN 3';
pause(0.2);
msgbx.String  = 'SELECTION COMPLETED...CLOSING IN 2';
pause(0.2);
msgbx.String  = 'SELECTION COMPLETED...CLOSING IN 1';
pause(0.2);
close all;

end