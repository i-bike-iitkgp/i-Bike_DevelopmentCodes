Step 1:Connect GSM modem with UART bridge

Step 2:Open arduino COM Port ( with appropriate port ID) , select baud rate of 38400 with carriage return 

Step 3:Type the following AT commands to enable appropriate settings for i-Bike
       
       Command 1:AT
       
       Return 1:OK
       
       Command 2:ATE0
       
       Return 2:OK
       
       Command 3:AT+CMGF=1
       
       Return 3:OK
       
       Command 4:AT+CNMI=1,2,0,0,0
       
       Return 4:OK

Step 4:Check the setting by sending a message to GSM module , and if it correctly returns +CMT statement with message then GSM 
       Module is ready to use for i-Bike
