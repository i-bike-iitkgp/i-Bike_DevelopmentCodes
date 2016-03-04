int count_nl=0;
int gsm_flag=0;
String gsm_inputString="";
String gsm_latilongistring="";
String gsm_string;
String gsm_lati="", gsm_longi="";
int gsm_move_status_flag=0;
float gsm_v_lati,gsm_v_longi;

void serialEvent3()
{
  while(Serial3.available()>0)
  {
    
    char inchar = (char)Serial3.read();
    gsm_inputString+= inchar; 
    delayMicroseconds(150);
     
     if(inchar == '\n')
     count_nl++;
     
     if (count_nl == 3)
     {
     gsm_string = gsm_inputString; 
     gsm_flag=1;
     Serial.println(gsm_inputString);
     count_nl=0;
     gsm_inputString="";
     }  //must print wanted string     
    
   }
}

void call_gsm( String input)
{
  
  if(gsm_flag == 1)   //inplies a message was received
  {
    int i,j,k;
    gsm_latilongistring="";
  for( i=0 ; input[i] != '\n'; i++)
  {
  }
  i++;
  for ( i>0 ; input[i] != '\n' ; i++)
  {
  }
  i++; 
  for (j=0 , i>0 ; input[i] != '\n' ; i++,j++)
  {
    gsm_latilongistring+=input[i];
  }
  //gsm_latilongistring[j]=',';
  Serial.println(gsm_latilongistring);  
  //Parsing lati longi
  for( j=0; gsm_latilongistring[j] !=',' ; j++)
  {
    gsm_move_status_flag = int(gsm_latilongistring[j])-48;  
  }
  j++;
  for( k=0 , j>0 ; gsm_latilongistring[j] !=',';j++,k++)
  {
    gsm_lati += gsm_latilongistring[j];
  }
  gsm_lati[k]='\0';
  j++;
  for( k=0 , j>0 ; gsm_latilongistring[j] !=',';j++,k++)
  {
    gsm_longi += gsm_latilongistring[j];
  }
  gsm_longi[k]='\0';
  j++;
  
  //converting to actual values
  gsm_v_lati=gsm_lati.toFloat();
  gsm_v_longi=gsm_longi.toFloat();
  
  Serial.print("lati=");Serial.println(gsm_v_lati,8);
  Serial.print("longi=");Serial.println(gsm_v_longi,8);
  gsm_flag=0;
  
  }
}

void modem_initialization(void)
{
  Serial3.print("AT\r");// Attention command to wake up GSM modem
  delay(500);
  Serial3.print("ATE0\r");
  delay(200);
  Serial3.print("AT+CMGF=1\r");      // setting up GSM in text mode
  delay(500);
  Serial3.print("AT+CNMI=1,2,0,0,0\r");      // setting up GSM in text mode
  delay(500);

}

void setup() {
  Serial.begin(38400);
  Serial3.begin(38400);
  gsm_inputString="";
  modem_initialization();
  
  // put your setup code here, to run once:

}

void loop() 
{
call_gsm(gsm_string);
Serial.println("lalallallallalalllalalallalallalallala");
}

