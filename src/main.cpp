#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <I2CKeyPad.h>     // NOVA BIBLIOTECLA DE I2C 
#include <F:\Documentos\Arduino\libraries\TM1637\TM1637Display.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <F:\Documentos\Arduino\libraries\Arduino-PID-Library-master\PID_v1.h>
#include <Preferences.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

template <typename T>
void copyVar_to_Var(T& Var1, T& Var2)      // Cópia o conteudo de uma variável simples para outra
{
       Var1 = Var2;
}

template <typename T, size_t N>
void copyVar_to_Var(T (&Var1)[N], T (&Var2)[N])     // Cópia o conteudo de uma variável tipo vetor para outra
{
    memcpy(Var1, Var2, N * sizeof(T));
}


bool Take_Semaphore(SemaphoreHandle_t Mutex)         // Tentar ober o semáforo e retorna true se conseguiu
{
      bool t = false;
      long oldMilis = millis();
      while (t == false)
      {
         t = xSemaphoreTake(Mutex,portMAX_DELAY);
         if (millis() - oldMilis > 500) // Espera 1/2 segundo senão conseguir aborta
            break;
         delay (20);
      }
      if (t)
          return true;
}

template <typename T>
void Take_Semaphore_and_Copy_Var(SemaphoreHandle_t Mutex, T& Var1, T& Var2)      // Pega o semáforo e copia o consteude de uma variavel local para uma global ou vice versa
{
      if (Take_Semaphore(Mutex))
      {
         copyVar_to_Var (Var1, Var2);
         xSemaphoreGive(Mutex);
      }
}


// VARIÁVEIS GLOBAIS
// VARIÁVEIS USADOS DEVIDO A NOVA BIBLIOTECA 12C DE TECLADO
#define deltaTime(val) (millis()-val)  
#define CLK 4                       // Module 7Seg connection pins (Digital Pins)
#define DIO 18                      // Module 7Seg connection pins (Digital Pins)
#define PINO_BOMBA  32              // Pino para ligar a Bomba de Recirculação
#define PINO_PWM  5
#define CANAL_PWM  0

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


unsigned long lastKeyPress = 0;    // Usado na rotina de telado
unsigned long keyDelay = 200;      // Usado na rotina de telado




LiquidCrystal_I2C lcd (0x27, 16, 2);
TM1637Display display(CLK, DIO); //PASSA OS PARÂMETROS PARA UMA FUNÇÃO DA BIBLIOTECA TM1637Display


I2CKeyPad keyPad(0x20);
BluetoothSerial SerialBT;

char keypad_layout[19] = "123A456B789C*0#DNF"; // N = NO_KEY, F = FAILED
Preferences preferences;

byte grau[8] ={ B00001100,                      // Simbolo de Grau para o LCD
                B00010010,
                B00010010,
                B00001100,
                B00000000,
                B00000000,
                B00000000,
                B00000000};


// ---------- Variáveis globais de troca de informações entre tasks

SemaphoreHandle_t ALARME_Mutex;
bool ALARME = false;
float LEITURA = 0 ;               // Temperatura Atual lida no termometro;
byte PIDINIT = 0;
byte SETPOINT;                    // Temperatura a ser alcançada - Usado nos Dois Cores
char  TECLA = ' ';                // Tecla pressionada no teclado
byte ETAPALCD = 0;                // Etapa da brassagem para exibição no LCD
byte ETAPA = 0;                   // Etapa da brassagem 
byte I_FLAG = 0;                    // Indica que deve exibir uma mensagem de brassagem no LCD
byte M_FLAG = 0;
int TIMER = 0;                    // Tempo de Mostura e Fervura
float POWER = 0;                  // Potencia instantânea aplicada na panela;
int POTPAN = 0;                // Potencia a ser usada na Panlea
double PANCONFIG[]  = { 8, 0.1, 0, 1, 50, 50};                       // Global de Parâmetros de Configuração do PID
byte TEMPERATURA[]  = { 70, 30, 40, 50, 60, 95, 0};                   // Global de Parâmetros de Temperatura de brassagem
byte TIMEPOINT[]    = { 10, 11, 12, 13, 14, 15,  9, 8, 7, 6, 5};      // Global de Parâmetros de Tempos de brassagem
byte PANPARAM[]     = { 1, 2, 1, 2, 3, 1, 2, 20 };                    // Global de Parâmetros gerais de brassagem



/////   ---------------------------------------------------------------------------------

//bool Take_Semaphore(SemaphoreHandle_t Mutex);
//void Take_Semaphore_and_Copy_Var(SemaphoreHandle_t Mutex, byte &Var1,  byte  Var2);

SemaphoreHandle_t LEITURA_Mutex;
SemaphoreHandle_t SETPOINT_Mutex;
SemaphoreHandle_t POTPAN_Mutex;
SemaphoreHandle_t ETAPALCD_Mutex;
SemaphoreHandle_t I_FLAG_Mutex;
SemaphoreHandle_t TECLA_Mutex;
SemaphoreHandle_t TIMER_Mutex;
SemaphoreHandle_t PARAM_Mutex;
SemaphoreHandle_t CONFIG_Mutex;
SemaphoreHandle_t ETAPA_Mutex;



// Protótipos de Funções
void Display7Seg(float Leitura);                                                                         // Escreve a temperatutra no Dispplay 7 segmentos
void EscreverLCD (int c, int l, String txt);                                                               // Escreve um texto na coluna c, linha l do LCD
char TecladoGetKey ();                                                                                     // retorna se alguma tecla foi pressionada
String TecladoLCDGetString (byte c, byte l, byte tam);                                                     // Entra com um texto pelo teclado com indicação no LCFD
byte Menu(byte col, byte lin);
void Tmenu (String txt, String val1, String val2);
int Menu_Get_Values(int Pot, String msg, int max, String unit);
void BombaOnOF (int &PotBomba, int pino_bomba);
void AlteraPotenciaPanela (int &PotPan);
void MostraPotenciaInstantanea ();
void PularEtapa (int &Etapa);
double Get_KpKiKd (double temp, String msg);
void Executa_SubMenu(double Panconfig[]);
void Entra_Parametros(String Text1, String Text2, byte &NumEnts, byte Lista1[], byte Lista2[], byte Lista3[], byte t);
void Menu_Parametros(byte PanParam[], byte Temperatura[], byte TimePoint[]);




// ---------------------------------------------------------------Rotinas de Inerface --------------------------------------------------------------------------


void LoadConfig (double PanConfig[], byte Temperatura[], byte TimePoint[], byte PanParam[])           // Salva as configurações da panela na flash
{
    preferences.begin("Config", true);
    PanParam[0] = preferences.getShort ("NR",PanParam[0]);
    PanParam[1] = preferences.getShort ("NA",PanParam[1]);
    PanParam[2] = preferences.getShort ("TA1",PanParam[2]);
    PanParam[3] = preferences.getShort ("TA2",PanParam[3]);
    PanParam[4] = preferences.getShort ("TA3",PanParam[4]);
    PanParam[5] = preferences.getShort ("TA4",PanParam[5]);
    PanParam[6] = preferences.getShort ("TA5",PanParam[6]);
    PanParam[7] = preferences.getShort ("NL",PanParam[7]);

    Temperatura[0] = preferences.getShort ("T0",Temperatura[0]);
    Temperatura[1] = preferences.getShort ("T1",Temperatura[1]);
    Temperatura[2] = preferences.getShort ("T2",Temperatura[2]);
    Temperatura[3] = preferences.getShort ("T3",Temperatura[3]);
    Temperatura[4] = preferences.getShort ("T4",Temperatura[4]);
    Temperatura[5] = preferences.getShort ("T5",Temperatura[5]);
    Temperatura[6] = preferences.getShort ("T6",Temperatura[6]);
    
    TimePoint[0] = preferences.getShort ("TP0",TimePoint[0]);
    TimePoint[1] = preferences.getShort ("TP1",TimePoint[1]);
    TimePoint[2] = preferences.getShort ("TP2",TimePoint[2]);
    TimePoint[3] = preferences.getShort ("TP3",TimePoint[3]);
    TimePoint[4] = preferences.getShort ("TP4",TimePoint[4]);
    TimePoint[5] = preferences.getShort ("TP5",TimePoint[5]);
    TimePoint[6] = preferences.getShort ("TP6",TimePoint[6]);
    TimePoint[7] = preferences.getShort ("TP7",TimePoint[7]);
    TimePoint[8] = preferences.getShort ("TP8",TimePoint[8]);
    TimePoint[9] = preferences.getShort ("TP9",TimePoint[9]);
    TimePoint[10] = preferences.getShort ("TP10",TimePoint[10]);
 
 
    PanConfig[0] = preferences.getDouble ("KP",PanConfig[0]);
    PanConfig[1] = preferences.getDouble ("KI",PanConfig[1]);
    PanConfig[2] = preferences.getDouble ("KD",PanConfig[2]);
    PanConfig[3] = preferences.getDouble ("TAMAIS",PanConfig[3]);
    PanConfig[4] = preferences.getDouble ("PBINIT",PanConfig[4]);
    PanConfig[5] = preferences.getDouble ("PPINIT",PanConfig[5]);
    preferences.end();
  
}


void SaveConfig (double PanConfig[], byte Temperatura[], byte TimePoint[], byte PanParam[])
{
  preferences.begin("Config", false);
  preferences.clear();

  preferences.putShort ("T0",Temperatura[0]);
  preferences.putShort ("T1",Temperatura[1]);
  preferences.putShort ("T2",Temperatura[2]);
  preferences.putShort ("T3",Temperatura[3]);
  preferences.putShort ("T4",Temperatura[4]);
  preferences.putShort ("T5",Temperatura[5]);
  preferences.putShort ("T6",Temperatura[6]);

  preferences.putShort ("TP0",TimePoint[0]);
  preferences.putShort ("TP1",TimePoint[1]);
  preferences.putShort ("TP2",TimePoint[2]);
  preferences.putShort ("TP3",TimePoint[3]);
  preferences.putShort ("TP4",TimePoint[4]);
  preferences.putShort ("TP5",TimePoint[5]);
  preferences.putShort ("TP6",TimePoint[6]);
  preferences.putShort ("TP7",TimePoint[7]);
  preferences.putShort ("TP8",TimePoint[8]);
  preferences.putShort ("TP9",TimePoint[9]);
  preferences.putShort ("TP10",TimePoint[10]);

  preferences.putShort ("NR",PanParam[0]);
  preferences.putShort ("NA",PanParam[1]);
  preferences.putShort ("TA1",PanParam[2]);
  preferences.putShort ("TA2",PanParam[3]);
  preferences.putShort ("TA3",PanParam[4]);
  preferences.putShort ("TA4",PanParam[5]);
  preferences.putShort ("TA5",PanParam[6]);
  preferences.putShort ("NL",PanParam[7]);

  preferences.putDouble ("KP",PanConfig[0]);
  preferences.putDouble ("KI",PanConfig[1]);
  preferences.putDouble ("KD",PanConfig[2]);
  preferences.putDouble ("TAMAIS",PanConfig[3]);
  preferences.putDouble ("PBINIT",PanConfig[4]);
  preferences.putDouble ("PPINIT",PanConfig[5]);
  preferences.end();
  delay (100);
}


void Display7Seg(float Leitura)                                                                         // Escreve a temperatutra no Dispplay 7 segmentos
{
   display.showNumberDecEx(Leitura*100, 0x40, 1, 4, 0);
}

void EscreverLCD (int c, int l, String txt)                                                               // Escreve um texto na coluna c, linha l do LCD
{
  lcd.setCursor(c,l);
  lcd.print(txt);
}

char TecladoGetKey ()                                                                                     // retorna se alguma tecla foi pressionada
{
  char ch =' ';
  if (keyPad.isPressed() && deltaTime(lastKeyPress) > keyDelay) {
    lastKeyPress = millis();
    ch = keyPad.getChar();
  }
  return ch;
}

String TecladoLCDGetString (byte c, byte l, byte tam)                                                     // Entra com um texto pelo teclado com indicação no LCFD
{
   char carac = ' ';
   int lin, col;
   String str = "";
   col = c;
   lin = l;
   lcd.cursor_on();
   lcd.setCursor(col, lin);
   boolean inicio = true;
   while (carac != '#')
   {
     char key = TecladoGetKey();
     if (key) 
        carac = key; 
      if (carac == '*')
         carac = '.';  
      if (carac != ' ' && carac != '#' && carac != '*' && carac != 'A' && carac != 'B' && carac != 'C' && carac != 'D' && str.length() < tam)
      {
          if (inicio)
          {
             for (byte c = 1; c <= tam; ++c)
                lcd.print(" ");
             lcd.setCursor(col, lin);
             inicio = false;
          }
          str = str + carac; 
          lcd.print(carac);
          carac = 0;
          ++ col;
      }
      if (carac != ' ' && carac == 'D')
      {
          if (str.length() > 0)
          {
            -- col;
            str.remove (str.length()-1,1);
            lcd.setCursor(col, lin);
            lcd.print(" ");
            lcd.setCursor(col, lin);
            carac = 0;
          }
      }
      if (str.length() == tam)
        lcd.setCursor(col-1, lin);
      delay (10);
   }
   lcd.setCursor(c, l);
   for (byte i = 1; i <= tam; i++) 
       lcd.print(" ");
   lcd.cursor_off();
//   beep();
   return str;
}


byte Menu(byte col, byte lin)
{
    const byte NLINES = 7;
    String MenuLines[][2] = 
    {
      {"Parametros", "Proporc.  - Kp"},
      {"Lig. Bomba", "Integral  - Ki"},
      {"Pot. Panela","Diferenc. - Kd"},
      {"Ver Pot. Inst.", "Temp. a mais"},
      {"Pular Etapa", "Pot Bomba Inic."},
      {"Config", "Pot Pan Inic."},  
      {"Sair do Menu", "Sair do Menu"}  
    };
    lcd.noCursor();
    byte mcont = lin;
    lcd.clear();
    EscreverLCD (0,0,"<"+MenuLines[mcont][col]+">");    
    EscreverLCD (1,1,MenuLines[mcont+1][col]);    
    while (1)
    {
        char tecla = TecladoGetKey ();
        if (tecla == 'A')
        {
            tecla = 0;
            if (mcont < NLINES - 1)
               ++mcont;    
            else 
                mcont = 0;
            lcd.clear();        
            EscreverLCD (0,0,"<"+MenuLines[mcont][col]+">");  
            if (MenuLines[mcont+1][col] != "")
               EscreverLCD (1,1,MenuLines[mcont+1][col]); 
            else  
               EscreverLCD (1,1,MenuLines[0][col]); 

        }
        if (tecla == 'B')
        {
            tecla = 0;
            if (mcont > 0)
               --mcont;    
            else 
                mcont = NLINES - 1;
            lcd.clear();        
            EscreverLCD (0,0,"<"+MenuLines[mcont][col]+">");    
            if (MenuLines[mcont+1][col] != "")
               EscreverLCD (1,1,MenuLines[mcont+1][col]); 
            else  
               EscreverLCD (1,1,MenuLines[0][col]); 
        }
        if (tecla == '#')
        {
            return mcont;
            break;
        }
        if (tecla == 'D')
        {
            return 6;
            break;
        }
        delay (30);
    }
    lcd.cursor();
}

void Tmenu (String txt, String val1, String val2)
{
    lcd.clear();
    EscreverLCD (0,0,txt);
    EscreverLCD (0,1,">");
    EscreverLCD (1,1,val1);
    EscreverLCD (11,1,val2);
}

int Menu_Get_Values(int Pot, String msg, int max, String unit)
{
  String txt = "";
  Tmenu (msg, "","");
  EscreverLCD (msg.length()+1,0,String(Pot)+unit);
  txt = TecladoLCDGetString (1, 1, 3);
  lcd.clear();
  if (txt != "")
  {  
     Pot = txt.toInt();
     if (Pot > max)
        Pot = max;
  }
  return Pot;
}

void BombaOnOF (int &PotBomba, int pino_bomba)
{
    PotBomba = Menu_Get_Values(PotBomba, "PotBomba:", 100, "%");
//    beep();
    if  (PotBomba == 0)
      digitalWrite(pino_bomba, LOW);
    else 
      digitalWrite(pino_bomba, HIGH);
}

void AlteraPotenciaPanela (int &PotPan)
{
    PotPan = Menu_Get_Values(PotPan, "PotPanela:", 100, "%");
}



void MostraPotenciaInstantanea ()
{
    float Potencia = 0; 
    lcd.clear();
    EscreverLCD (0,0,"Pot_Inst:"); 
    char tecla = '\0';
    while (tecla != '#')
    {
      tecla = TecladoGetKey ();
      Take_Semaphore_and_Copy_Var(POTPAN_Mutex,Potencia, POWER);                          // Salva a Potencia da variável global POWER na local
      EscreverLCD (10,0,"     "); 
      EscreverLCD (10,0,String (Potencia)+"%"); 
      delay (100);
    }
}


void PularEtapa (byte &Etapa)
{
    byte ret = Menu_Get_Values(Etapa, "Etapa:", 5, "");
    if ( ret <= 3)
        Etapa = ret;
    else
        Etapa = 3;
}


double Get_KpKiKd (double temp, String msg)
{
  String txt = "";
  Tmenu (msg, "","");
  EscreverLCD (msg.length()+1,0,"("+String(temp)+")");
  txt = TecladoLCDGetString (1, 1, 3);
  if (txt != 0)
    temp = txt.toFloat();
  lcd.clear();
  return temp;
}

void Executa_SubMenu(double Panconfig[])
{
    bool sair = false;
    byte ret_menu = 0;    
    while (!sair)
    {
        ret_menu = Menu(1,ret_menu);
        
        lcd.clear();
        switch (ret_menu)
        {
          case 0:  // Altera KP
          { 
                Panconfig[0] = Get_KpKiKd (Panconfig[0], "Entre KP"); break;
          }
          case 1:  //  Altera Ki
          { 
                Panconfig[1] = Get_KpKiKd (Panconfig[1], "Entre KI"); break;
          }
          case 2:  // Altera Kd
          { 
                Panconfig[2] = Get_KpKiKd (Panconfig[2], "Entre KD"); break;
          }
          case 3:  // Altera temperatura a mais na mostura
          { 
                Panconfig[3] = Get_KpKiKd (Panconfig[3], "Temperatura"); break;
          }
          case 4:  // Altera potencia inicial da bomba
          { 
                Panconfig[4] = Get_KpKiKd (Panconfig[4], "Pot Bomba"); break;
          }
          case 5:  // Altera potencia inicial da bomba
          { 
                Panconfig[5] = Get_KpKiKd (Panconfig[5], "Pot Panela"); break;
          }
          case 6:  // sair
          { 
                sair = true;                              
          }
        }
        delay (10);
    }

}



void Entra_Parametros(String Text1, String Text2, byte &NumEnts, byte Lista1[], byte Lista2[], byte Lista3[], byte t)
{
  byte i = 0;
  Tmenu (Text1, String(NumEnts),"");
  String txt = TecladoLCDGetString (1, 1, 1);
  if (txt != "")
  {    
    NumEnts = txt.toInt();
    if (NumEnts == 0) NumEnts = 1;
    if (NumEnts > 5)  NumEnts = 5;
  }
  while (i < NumEnts)
  {
     if  (t ==  0)
     {
          Tmenu (Text2+" "+String(i+1),String(Lista1[i])+String((char)223),String(Lista2[i])+" '");
          txt = TecladoLCDGetString (1, 1, 2);
          if (txt == "")
              txt = String(Lista1[i]);
          Lista1[i] = txt.toInt();
          EscreverLCD (1,1,txt); 
          txt = TecladoLCDGetString (11, 1, 2);
          if (txt == "")
              txt = String(Lista2[i]);
          Lista2[i] = txt.toInt(); 
          Serial.print (Lista1[i])  ;
          Serial.print (" - ")  ;
          Serial.println (Lista2[i])  ;
     }
     else if  (t ==  1)
     {
         Tmenu (Text2+" "+String(i+1),String(Lista2[i+6])+" '"+" Tipo",String(Lista3[i+2]));
          txt = TecladoLCDGetString (1, 1, 2);
          if (txt == "")
              txt = String(Lista2[i+6]);
          Lista2[i+6] = txt.toInt();
          EscreverLCD (1,1,txt); 
          txt = TecladoLCDGetString (11, 1, 2);
          if (txt == "")
              txt = String(Lista3[i+2]);
          Lista3[i+2] = txt.toInt();    
     }
     ++i;
     delay(10);
  }    

}


void Menu_Parametros(byte PanParam[], byte Temperatura[], byte TimePoint[])
{
  byte NumRampas = PanParam[0];
  byte NumAdics = PanParam[1];
  Entra_Parametros("Rampas (Max 5)", "Rampa", NumRampas, Temperatura, TimePoint, PanParam, 0);  
  Entra_Parametros("Adicoes (Max 5)", "Adicao",NumAdics, Temperatura, TimePoint, PanParam, 1);  
  PanParam[0] = NumRampas;
  PanParam[1] = NumAdics;
  
  Tmenu ("Fervura", String(Temperatura[5])+String((char)223)+" Tempo",String(TimePoint[5])+"'");
  String txt = TecladoLCDGetString (1, 1, 2);
  if (txt == "")
    txt = String(Temperatura[5]);
  EscreverLCD (1,1,txt); 
  Temperatura[5] = txt.toInt();     
  txt = TecladoLCDGetString (11, 1, 2);
  if (txt == "")
    txt = String(TimePoint[5]);
  TimePoint[5] = txt.toInt();     
  lcd.clear();
 }


// -------------------------- TASK DE ROTINAS DE INTERFACE ----------------------------------------------------------------------------------------------------

void loop3(void *z)      
{
  byte EtapaLCD = 0;                // Indica etapa de informação do LCD. Recebe de ETAPALCD
  byte i_flag = 0;                    // Indica etapa de informação da flag de escrita no LCD. Recebe de I_FLAG
  char tecla = ' ';                 // Indica etapa de informação da TECLA PRESSIONADA. Altera TECLA
  float Leitura = 0;                // Indica etapa de informação da temperatura. Recebe de TEMPERATURA
  float LeituraAnterior = 0;        // armazena a temperatura anterior para comparar com a atual
  int PotBomba = 80;
  int PotPan = 0;;
  byte EtapaAtual = 0;
  int timer = 0;
  int old_timer = 0;
  bool Alarme = false;
// ---------- Variáveis de Mensagem de LCD
  
  String MensagemLCDL1[] = {"Easy Brew - V3",        // 0       
                            "Aquecendo...",          // 1  
                            "Adicione o Malte",      // 2
                            "Mosturando...",         // 3
                            "Fim de Mostura",        // 4
                            "Fervura Atingida",      // 5
                            "Fervendo...",           // 6
                            "Fim de Fervura",        // 7
                            "Adicione o Lupulo",     // 8
                            "Adicione o Wirfloc",    // 9
                            "Adicione o Adjunto",    // 10
                            "Fim da Brassagem",      // 11
                            };

  String MensagemLCDL2[] = {"",                       // 0
                            "Setpoint:",              // 1
                            "Tecla # p/ cont." ,      // 2
                            "Tempo:",                 // 3
                            "Tecla # p/ cont." ,      // 4
                            "Tecla # p/ cont.",       // 5
                            "Tempo:",                 // 6
                            "# Desl - * Cont.",       // 7
                            "Tecla # p/ Cont.",       // 8
                            "Tecla # p/ Cont.",       // 9
                            "Tecla # p/ Cont.",       // 10
                            "Tecla # p/ Cont."};      // 11


  double PanConfig[]  = { 0,                   // Kp
                          0,                  // Ki
                          0,                   // Kd
                          0,                   // Temperatura amais na rampa inicial
                          50,                  // Potencia inicial da bomba
                          50};                 // PPinit - Potencia Inicial da Panela

  byte Temperatura[] = { 70,                   // Temperatura da Rampa 1
                          30,                  // Temperatura da Rampa 2
                          40,                  // Temperatura da Rampa 3
                          50,                  // Temperatura da Rampa 4
                          60,                  // Temperatura da Rapma 5
                          95,                  // Temperatura usadas na mostura e fervura 
                          0};                  // Temperatura a mais na adicção do malte (primeira Rampa)

  byte TimePoint[]   = { 10,                   // Tempo da Rampa 1
                          11,                  // Tempo da Rampa 2
                          12,                  // Tempo da Rampa 3
                          13,                  // Tempo da Rampa 4
                          14,                  // Tempo da Rampa 5
                          15,                  // Tempo da Fervura
                          9,                   // Tempo da Adição 1
                          8,                   // Tempo da Adição 2
                          7,                   // Tempo da Adição 3 
                          6,                   // Tempo da Adição 4 
                          5};                  // Tempo da Adição 5

    byte PanParam[] = { 1,                     // Numero de Rampas
                        2,                     // Numero de Adições na fervura
                        1,                     // Tipo da Adição 1
                        2,                     // Tipo da Adição 2
                        3,                     // Tipo da Adição 3
                        1,                     // Tipo da Adição 4
                        2,                     // Tipo da Adição 5
                        20};                   // Volume da brassagem 
  
  Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);
  Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura,TEMPERATURA);
  Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint,TIMEPOINT);
  Take_Semaphore_and_Copy_Var(CONFIG_Mutex, PanConfig, PANCONFIG);

  PotPan = PanConfig[5];
  Take_Semaphore_and_Copy_Var(POTPAN_Mutex, POTPAN, PotPan);

  EscreverLCD (1,0,MensagemLCDL1[EtapaLCD]);
  while (1)
  {

      tecla = TecladoGetKey ();
      if (tecla != ' ')                                                                         // Se foi e diferente de ' '
      {
         Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);   
      }
      if (tecla ==  'A')
      {
          tecla = ' ';
          lcd.cursor_on();
          bool sair = false;
          byte ret_menu = 0;
          while (!sair)
          {
              ret_menu = Menu(0,ret_menu);
              switch (ret_menu)
              {
                  case 0: {
                            Menu_Parametros(PanParam, Temperatura, TimePoint);
                            tecla = ' ';
                            Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                            break;
                          }
                  case 1: BombaOnOF (PotBomba, PINO_BOMBA);  break;
                  case 2: AlteraPotenciaPanela (PotPan);  break;
                  case 3: MostraPotenciaInstantanea (); break;
                  case 4: PularEtapa (EtapaAtual); sair = true; break;   
                  case 5: Executa_SubMenu(PanConfig); break;
                  case 6: { lcd.clear(); sair = true;} 
              } 
              delay(30);
          }
          SaveConfig (PanConfig,Temperatura, TimePoint, PanParam);
          Serial.println (PanParam[0]);
          Serial.println (PanParam[1]);
          i_flag = 1;
          Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, i_flag);
          Take_Semaphore_and_Copy_Var(POTPAN_Mutex, POTPAN, PotPan);
          Take_Semaphore_and_Copy_Var(ETAPA_Mutex, ETAPA, EtapaAtual);
          Take_Semaphore_and_Copy_Var(PARAM_Mutex, PANPARAM, PanParam);
          Take_Semaphore_and_Copy_Var(PARAM_Mutex, TEMPERATURA, Temperatura);
          Take_Semaphore_and_Copy_Var(PARAM_Mutex, TIMEPOINT, TimePoint);
          Take_Semaphore_and_Copy_Var(CONFIG_Mutex, PANCONFIG, PanConfig);
          byte m_flag = 1;
          Take_Semaphore_and_Copy_Var(PARAM_Mutex, M_FLAG, m_flag);
      }
      if (tecla ==  'B')
      {
          Alarme = false;
          Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
      }
      tecla = ' ';
      Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, i_flag, I_FLAG);                              // Monitora se a flag de escrita na tela do LCD está setada
      if (i_flag == 1)
      {
          Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, EtapaLCD, ETAPALCD);              // Se sim pela a etapa lcd e escrve na tela, as linhas 1 e 2
          Serial.println (EtapaLCD);
          lcd.clear();
          EscreverLCD (0, 0, MensagemLCDL1[EtapaLCD]);
          EscreverLCD (0, 1, MensagemLCDL2[EtapaLCD]);
          i_flag = 0;
          Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, i_flag);
      }
      Take_Semaphore_and_Copy_Var(LEITURA_Mutex, Leitura, LEITURA);                     // Pega temperatura atual e escrve do display 7 segmentos
      if (EtapaLCD == 1)
      {
            byte Setpoint;
            Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, Setpoint, SETPOINT);
            EscreverLCD (10, 1, String(Setpoint)+(char)0 + "C");                        // (char) 0 se refere ao simbolo de grau criado na seção de Setup 


      }
      if (EtapaLCD == 3 || EtapaLCD == 6)
      {
          lcd.cursor_off();
          Take_Semaphore_and_Copy_Var(TIMER_Mutex, timer, TIMER);                     // Pega temperatura atual e escrve do display 7 segmentos
          if (timer != old_timer)
          {
              EscreverLCD (7, 1, "    ");
              EscreverLCD (7, 1, String (timer / 60 + 1));
              old_timer = timer;
          }
      }  
      if (Leitura != LeituraAnterior)
      {
          Display7Seg (Leitura);                                                                // Se a temperatura foi alterada, escreve no 7 segmentos
          LeituraAnterior = Leitura;
      }   
      delay(10);
  }     
}


// -------------------------- TASK DE MOSTURAÇÃO ----------------------------------------------------------------------------------------------------
void loop2(void *z)     
{
 
bool InitVar = false;             // Usado somete para inicializar as variáveis do processo
char tecla = ' ';                 // tecla pressionada no teclado. recebe de TECLA
float Leitura = 95;               // Variavel local de temperatura. Recebe de LEITURA
byte Etapa = 0;                   // etapa atual do processso
byte EtapaLCD = 0;                // Etapa atual do LCD. Usada para passar para ETAPALCD
byte SetPoint = 0;                // Temperartura a ser atingida
byte RampCont = 0;                // Contatdor de rampas de barassagem
long HoraIniEtapa = 0;            // Usada na contagem de tempo
bool InProrrog = false;           // Usada para indicar que o processo está em prorrogção de fervura
byte m_flag = 0;                  // Recebe a fla de indicação que houve uma possível alteração nos parâmetros de mosturação - Recebe de M_FLAG
byte Temp_a_Mais = 0;                  // Temperatura a ais na primeira rampabool Alarme = false;
bool Alarme = false;



// ---------- Variáveis de parâmetros de brassagem

byte Temperatura[] = {0,                    // Temperatura da Rampa 1
                       0,                    // Temperatura da Rampa 2
                       0,                    // Temperatura da Rampa 3
                       0,                    // Temperatura da Rampa 4
                       0,                    // Temperatura da Rapma 5
                       0,                    // Temperatura usadas na fervura  
                       0};                    // Temperatura a mais na adicção do malte (primeira Rampa)
byte TimePoint[]   = {0,                     // Tempo da Rampa 1
                       0,                     // Tempo da Rampa 2
                       0,                     // Tempo da Rampa 3
                       0,                     // Tempo da Rampa 4
                       0,                     // Tempo da Rampa 5
                       0,                     // Tempo da Fervura
                        0,                    // Tempo da Adição 1
                        0,                    // Tempo da Adição 2
                        0,                    // Tempo da Adição 3 
                        0,                    // Tempo da Adição 4 
                        0};                   // Tempo da Adição 5
    byte PanParam[] = { 0,                      // Numero de Rampas
                        0,                      // Numero de Adições na fervura
                        0,                      // Tipo da Adição 1
                        0,                      // Tipo da Adição 2
                        0,                      // Tipo da Adição 3
                        0,                      // Tipo da Adição 4
                        0,                       // Tipo da Adição 5
                        0};                    // Volume da brassagem                       

  Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);
  Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura,TEMPERATURA);
  Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint,TIMEPOINT);

  while (1)
  {
    if (InitVar == false)
    {
        tecla =  ' ';
        Etapa = 0;
        EtapaLCD = 0;
        SetPoint = 0; // Temperartura a ser atingida
        RampCont = 0;
        HoraIniEtapa = 0;
        InProrrog = false;
        InitVar = true;
    }
    switch (Etapa)    
    {
      case 0:  
      {
            byte etlcd = 0;
            byte iflag = 1;
            Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);
            Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                       // Altera a tela do LDC 16x 2
            while (Etapa == 0)                                                              // Aguarda a tecla # ser pressionado para iniciar o aquecimento da panela e partir para etapa 1
            {

                Take_Semaphore_and_Copy_Var(PARAM_Mutex, m_flag, M_FLAG);                   // Se a flag for 1, hove alguma alteração nos parametros de mostura
                if (m_flag == 1)
                {
                    Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);           // 
                    Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura, TEMPERATURA);
                    Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint, TIMEPOINT);
                    Take_Semaphore_and_Copy_Var(ETAPA_Mutex, Etapa, ETAPA);
                    byte m_flag = 0;
                    Take_Semaphore_and_Copy_Var(PARAM_Mutex, M_FLAG, m_flag);                    // Zera a flag
                }

                Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);                     // Monitora se alguma tecla foi pressionada
                if (tecla == '#')                                                                   // Se foi pressionada a tecla #
                {
                      tecla = ' ';
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);               // Zera a variavel global TECLA
                      SetPoint = Temperatura[0] + int(Temperatura[6]*PanParam[7]/20);       // Setpoint  da rampa inicial + um adicional de no máximo x graus  para adicionar o malte
                      Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);      // Salva o Setpoint na Variável Global SETPOINT;                     
                      byte pidinit = 1;
                      Take_Semaphore_and_Copy_Var(POTPAN_Mutex, PIDINIT, pidinit);                // Seta a Flag de PIDINIT para ligar o PID;                     
                      Etapa = 1;                                                                    // Vai para a Etapa 1
                }
                delay (10);
            }
      }
      break;
      case 1:
      {     //Aquece á água até a temperatura da rampa atual

            byte etlcd = 1;
            byte iflag = 1;            
            Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                           // Altera a tela do LDC 16x 2
            Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                       // Seta a flag indicando que ocorreu alteração
            while (Etapa == 1)
            {

                  Take_Semaphore_and_Copy_Var(PARAM_Mutex, m_flag, M_FLAG);                 // Se a flag for 1, hove alguma alteração nos parametros de mostura
                  if (m_flag == 1)
                  {
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);         // 
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura, TEMPERATURA);
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint, TIMEPOINT);
                      byte m_flag = 0;
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, M_FLAG, m_flag);                  // Zera a flag
                  }


                  Take_Semaphore_and_Copy_Var(LEITURA_Mutex, Leitura, LEITURA );            // Pega a temperatura atual
                  SetPoint = Temperatura[RampCont] +  Temp_a_Mais;       // Setpoint  da rampa inicial + um adicional de no máximo x graus  para adicionar o malte
                  Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);      // Salva o Setpoint na Variável Global SETPOINT;                     
				  
                  if (Leitura >= SetPoint)                                                          //Atingiu a temperartura do Setpoint
                  {
                      Alarme = true;
                      Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                      if (RampCont < 5)                                                             // Executando aquecimento para as rampas de temperatura
                      {
                          if (RampCont == 0)
                          {
                              byte etlcd = 2;
                              byte iflag = 1;  
                              Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);
                              Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);
                          }
                          Etapa = 2;
                      }
                      else                                                                          // finalizou as rampas de temperatura
                      {
                          Serial.println ("Aqui 3");
                          tecla = ' ';
                          Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                          byte etlcd = 5;
                          byte iflag = 1;            
                          Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);
                          Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);
                           while (tecla != '#')                                                          // Aguarada a Tecla # ser pressionada
                           {
                              Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);          // Monitora se alguma tecla foi pressionada
                              delay (50);
                           } 
                           Etapa = 3;
                           SetPoint = 100;                                                       // Setpoint  da Fervura
                           Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);      // Salva o Setpoint na Variável Global SETPOINT;                     
                      }
                      if (RampCont == 0)
                      {
                         tecla = ' ';
                         Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                         while (tecla != '#')                                                          // Aguarada a Tecla # ser pressionada
                          {
                              Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);          // Monitora se alguma tecla foi pressionada
                              delay (50);
                          } 
                      }
                      if (Etapa == 3)
                      {
                          SetPoint = 100;                                                       // Setpoint  da Fervura
                          Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);      // Salva o Setpoint na Variável Global SETPOINT;                     
                      }   
                      Temp_a_Mais = 0;                                                             // Zera a temperatura a mais pois Finalizou a primeira Rampa

                      tecla = ' ';                                                                  
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);               // Zera a variavel global TECLA
                      HoraIniEtapa = time(NULL);                                                    // Pega o tempo atual
                  }

                  delay (10);
            }

      }
      break;
      case 2:   // Executa a rampa de temperatura
      {
           byte etlcd = 3;
           byte iflag = 1;            
           Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                                    // Seta a flag indicando que ocorreu alteração
           Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                                        // Altera a tela do LDC 16x 2
           while (Etapa == 2)
            {
                  Take_Semaphore_and_Copy_Var(PARAM_Mutex, m_flag, M_FLAG);                             // Se a flag for 1, hove alguma alteração nos parametros de mostura
                  if (m_flag == 1)
                  {
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);                     // 
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura, TEMPERATURA);
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint, TIMEPOINT);
                      byte m_flag = 0;
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, M_FLAG, m_flag);                         // Zera a flag
                  }

                  SetPoint = Temperatura[RampCont] +  Temp_a_Mais;                                      // Setpoint  da rampa inicial + um adicional de no máximo x graus  para adicionar o malte
                  Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);                      // Salva o Setpoint na Variável Global SETPOINT;                     

                  int Timer = TimePoint[RampCont] * 60 - (time(NULL) - HoraIniEtapa);                           // Calcula o tempo restante
                  Take_Semaphore_and_Copy_Var(TIMER_Mutex, TIMER, Timer);                               // Salva o tempo restante na varável TIMER
                  if (Timer == 0)    // Finalizou a rampa
                  {
                        if (RampCont < PanParam[0] - 1)                                                           // Se ainda há outras rampas a serem executadas
                        {
                            ++RampCont;                                                                         // Executa a próxima rampa
                            Etapa = 1;                                                                          // Volta para a etapa de aquecimento
                        } 
                        else                                                                                   // Ou se finalizou a execução das rampas parte para a rampa de fervura
                        { 
                              Alarme = true;
                              Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                              byte etlcd = 4;
                              byte iflag = 1;            
                              Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                 // Seta a flag indicando que ocorreu alteração
                              Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                         // Altera a tela do LDC 16x 2 
                              RampCont = 5;
                              Etapa = 1;                                                                        // Volta para a etapa de aquecimento
                              tecla = ' ';
                              Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                              while (tecla != '#')    // Aguarada a Tecla # ser pressionada
                              {
                                 Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);                // Monitora se alguma tecla foi pressionada
                                 delay (50);
                              } 
                              Alarme = false;
                              Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                              tecla = ' ';
                              Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);                    // Zera a variavel global TECLA
                        }
                        SetPoint = Temperatura [RampCont];
                        Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, SETPOINT, SetPoint);                // Salva o Setpoint na Variável Global SETPOINT;                     
						            HoraIniEtapa = 0;
                 }
                  delay (500);

            }

      }
      break;
      case 3:   // Executa a fervura e adições
      {
            HoraIniEtapa = time(NULL);
            byte contAdic = 0;
            byte etlcd = 6;
            byte iflag = 1;            
            int Timer = 10000;
            Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                                      // Seta a flag indicando que ocorreu alteração
            Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag); 
            while (Etapa == 3)
            {
                  Take_Semaphore_and_Copy_Var(PARAM_Mutex, m_flag, M_FLAG);                               // Se a flag for 1, hove alguma alteração nos parametros de mostura
                  if (m_flag == 1)
                  {
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, PanParam, PANPARAM);                       // 
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, Temperatura, TEMPERATURA);
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, TimePoint, TIMEPOINT);
                      byte m_flag = 0;
                      Take_Semaphore_and_Copy_Var(PARAM_Mutex, M_FLAG,  m_flag);                                // Zera a flag
                  }

                  if (Timer > 0)
                      Timer = TimePoint[5] * 60 - (time(NULL) - HoraIniEtapa);                                     // Calcula o tempo restante
                  Take_Semaphore_and_Copy_Var(TIMER_Mutex, TIMER, Timer);                                  // Salva o tempo restante na varável TIMER
                  Serial.print(TimePoint[6+contAdic] * 60);                                       
                  Serial.print(" - ");                                       
                  Serial.println(Timer);                                       
                  if (Timer == TimePoint[6+contAdic] * 60 && contAdic < PanParam[1] && InProrrog == false)             // Se o timer chegou em algum ponto de adição durante a fervura 
                  {
                      Alarme = true;
                      Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                      Serial.print("ContAdic = ");
                      Serial.println(contAdic +2);
                      etlcd = 8 + PanParam[contAdic+2];
                      Serial.print("PanParam[contAdic] = ");
                      Serial.println(PanParam[contAdic+2]);
                      iflag = 1;  
                      Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                            // Altera a tela do LDC 16x 2  
                      Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                                    // Seta a flag indicando que ocorreu alteração 
                      tecla = ' ';
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                      while (tecla != '#')    // Aguarada a Tecla # ser pressionada
                      {
                          Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);                           // Monitora se alguma tecla foi pressionada  
                      } 
                      tecla = ' ';
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);                               // Zera a variavel global TECLA 
                      Alarme = false;
                      Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                      etlcd = 6;
                      iflag = 1;            
                      Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                         // Seta a flag indicando que ocorreu alteração
                      Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag); 
                      ++contAdic;                                                                                                       // Inccrementa o contador de adição
                      Serial.println("Aqui 1");
                  }
                  if (Timer <= 0)                                                                                   // Finalizou a rampa
                  {
                      Serial.println("Aqui 2");
                      byte etlcd = 7;
                      byte iflag = 1;            
                      Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                             // Altera a tela do LDC 16x 2
                      Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                                     // Seta a flag indicando que ocorreu alteração  
                      Alarme = true;
                      Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                      tecla = ' ';
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                      while (tecla != '#'  && tecla != '*')    // Aguarada a Tecla # ser pressionada
                      {
                          Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA);   
                          delay (50);                        // Monitora se alguma tecla foi pressionada 
                      } 
                      Alarme = false;
                      Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
                      if (tecla == '#')                                                                             // Continua para o fim da brassagem
                      { 
                          Etapa = 4;
                          tecla = ' ';
                          Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);                           // Zera a variavel global TECLA
                      }
                      else if (tecla == '*')                                                                        // Prorroga a fervura
                      {
                          
                          // Implementar aqui a entrada do tempo a mais
                          TimePoint[5] = 10;                                                                         // tempo a mais de fervura
                          Etapa = 3;
                          InProrrog = true;
                          HoraIniEtapa = 0;
                          tecla = ' ';
                          Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);                           // Zera a variavel global TECLA  
                          break;
                      }
                  }
                  delay (500);
            }
      }
      break;
      case 4:   // Final da Brassagem
      {
           byte etlcd = 11;
           byte iflag = 1;            
            Take_Semaphore_and_Copy_Var(ETAPALCD_Mutex, ETAPALCD, etlcd);                                    // Altera a tela do LDC 16x 2
            Take_Semaphore_and_Copy_Var(I_FLAG_Mutex, I_FLAG, iflag);                                             // Seta a flag indicando que ocorreu alteração 
            while (Etapa == 4)
            {
                  delay (10);
                  tecla = ' ';
                  Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);
                  while (tecla != '#' )    // Aguarada a Tecla # ser pressionada
                  {
                      Take_Semaphore_and_Copy_Var(TECLA_Mutex, tecla, TECLA); 
                      delay (50);                             // Monitora se alguma tecla foi pressionada
                  } 
                  tecla = ' ';
                  Take_Semaphore_and_Copy_Var(TECLA_Mutex, TECLA, tecla);                                 // Zera a variavel global TECLA
                  Etapa = 0;                                                                                      // Volta para a Etapa 0 e fica aguardando outra brassagem
                  tecla = 0;
                  InitVar = false;                                                                                // Indica que as variáveis do processo devem ser resetadas aos valores padrão
            }
      }
      break;
    }   
    delay (10);
  }    
}

// ********************* ROTINAS DO PID *******************************
#define CANAL_PWM  0



//Instacia o Objeto oneWire e Seta o pino do Sensor para iniciar as leituras
const int oneWireBus = 23;
OneWire oneWire(oneWireBus);
//Repassa as referencias do oneWire para o Sensor Dallas (DS18B20)
DallasTemperature sensor(&oneWire);



void loop1(void *z)
{

  float Power = 0;
  float Temp_lida = 0;
  float oldPower =0;
  float MaxPower = 0;  // Potencia máxima a ser usada.
  float OldMaxPower = 0;  //
  // Variáveis locaid do Controle PID                       
  double kp ;  //20 Litros
  double ki ;   //20 Litros
  double kd ;
  double Setpoint = 0;
  byte Setp = 0;
  int PIDlimMin = 10;
  int PIDlimMax = MaxPower;
  double Input, Output;
  long OldMilis = 0;
  sensor.begin();
  byte IniciarPID = 0;
  int PotPan = 0;
  float Leitura = 0;
 
  PID pid(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
  pid.SetMode(AUTOMATIC); 
  pid.SetOutputLimits(PIDlimMin, PIDlimMax);
  
  ledcWrite(CANAL_PWM, map (Power, 100, 0 , 0, 1000 ));

  double PanConfig[]  = { 0,                   // Kp
                          0,                   // Ki
                          0,                   // Kd
                          0,                   // Temperatura a mais na rampa inicial
                          50,                  // Potencia inicial da bomba
                          100};                 // PPinit - Potencia Inicial da Panela


  while (1)
  {
       if (IniciarPID == 0)
       {
           while (IniciarPID == 0)
           {
                sensor.requestTemperatures(); 
                Temp_lida = sensor.getTempCByIndex(0);
                if (Temp_lida > 0)
                  Input = Temp_lida;
                float Leitura = Input;
                Take_Semaphore_and_Copy_Var(LEITURA_Mutex, LEITURA, Leitura);
                delay (100);
                Take_Semaphore_and_Copy_Var(POTPAN_Mutex, IniciarPID, PIDINIT);
                if (IniciarPID == 1)
                {
                      Serial.println("Iniciando o PID...");
                      Take_Semaphore_and_Copy_Var(POTPAN_Mutex, PotPan, POTPAN);
                      Take_Semaphore_and_Copy_Var(CONFIG_Mutex, PanConfig, PANCONFIG);
                      MaxPower = PotPan;
                      PIDlimMin = 0;
                      kp = PanConfig[0];  
                      ki = PanConfig[1];  
                      kd = PanConfig[2];
                      OldMaxPower = MaxPower;
                      Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, Setp, SETPOINT);
                      Setpoint = Setp;
                      pid.SetOutputLimits(PIDlimMin, MaxPower);
                      pid.SetTunings(kp, ki, kd);
                }  
           }    
       }  
       if ((millis() - OldMilis) > 300)   // EXECUTA ESSA PARTE 3 VEZES POR SEGUNDO
       {
    //        sensor.requestTemperatures(); 
            if (Serial.available() > 0) 
            {
                  String txt = Serial.readString();
                  Temp_lida = txt.toFloat();
             }    
     //       Temp_lida = sensor.getTempCByIndex(0);
            if (Temp_lida > 0)
               Input = Temp_lida; 
            Leitura = Input;   
            Take_Semaphore_and_Copy_Var(POTPAN_Mutex, PotPan, POTPAN);
            Take_Semaphore_and_Copy_Var(LEITURA_Mutex, LEITURA, Leitura);
            Take_Semaphore_and_Copy_Var(SETPOINT_Mutex, Setp, SETPOINT);
            Take_Semaphore_and_Copy_Var(POTPAN_Mutex, POWER, Power);
            Take_Semaphore_and_Copy_Var(CONFIG_Mutex, PANCONFIG, PanConfig);
            MaxPower = PotPan;
            PIDlimMin = 0;
            Setpoint = Setp;
            kp = PanConfig[0];  
            ki = PanConfig[1];  
            kd = PanConfig[2];
            if (MaxPower != OldMaxPower)    // Se o valor da potencia máxima foi alterado
            {
                pid.SetOutputLimits(PIDlimMin, MaxPower);
                OldMaxPower = MaxPower;
            }   
            if (Input > 0)
                pid.Compute();    // Caclula a potencia
            Power = Output;   // Atribui a power;
            ledcWrite(CANAL_PWM, map (Power, 100, 0 , 0, 1000 )); 

 /*              Serial.println ("Alterada a potência calculada... Imprimindo parâmetros atuais");
                Serial.print ("Kp= ");
                Serial.print (pid.GetKp());
                Serial.print (" Ki= ");
                Serial.print (pid.GetKi());
                Serial.print (" Kd= ");
                Serial.print (pid.GetKd());
                Serial.print (" Temperatura ");
                Serial.print(Input);
                Serial.print ("ºC ");
                Serial.print ("Potencia: ");
                Serial.print (Power);
                Serial.print (" Setpoint: ");
                Serial.print (Setpoint);
                Serial.print (" Potencia Máxima: ");
                Serial.println (MaxPower);        
                Serial.println(Input);
                Serial.println(" ");   */

            OldMilis = millis();   
       }  
       delay (30);  
  }   
}



// ---------------------------------------------------------------Inicio da Seção de Alarme --------------------------------------------------------------------------


#define BUZZER 19

void loop4(void *z)
{
 
  bool Alarme = false;
  int cont = 0;
  // Configure BUZZER functionalities.
    ledcSetup(BUZZER, 300, 10);
    ledcAttachPin(BUZZER, 0);
  // Attach BUZZER pin.
 // ledcAttach(BUZZER,300, 10);

  while (1)
  {

      Take_Semaphore_and_Copy_Var(ALARME_Mutex, Alarme, ALARME);
      if (Alarme)
      {
            ++ cont;
            if (cont == 20)
            {
               Alarme = false;
               cont = 0;
               Take_Semaphore_and_Copy_Var(ALARME_Mutex, ALARME, Alarme);
               ledcWrite(BUZZER,0);
            }

            int toneval = 100;
            for (byte cont = 1; cont<= 50; ++cont)
            {
              ledcWriteTone(BUZZER,toneval + cont*5);
              delay (20);
            }
      }
      else
           ledcWrite(BUZZER,0);

      delay (100);  
  }   
}


// ---------------------------------------------------------------Inicio da Seção de Bluetooth --------------------------------------------------------------------------

/*
String BluetoothSetConfig()
{
	String saida = "";
	for (int i = 0; i <= 4; i++)
  {
    if (Temperatura[i] < 10)
		   saida = saida + "0"+ String(Temperatura[i]);   // Temperatura das rampas
    else 
      saida = saida + String(Temperatura[i]);
  }
	for (int i = 0; i <= 4; i++)
	{	
		if (TimePoint[i] < 10)
		   saida = saida + "0" + String(TimePoint[i]);
		else saida = saida + String(TimePoint[i]);
	}
  if (TimePoint[9] < 10) 
     saida = saida + "0" + String(TimePoint[9]);   // Tempo de Fervura
  else saida = saida + String(TimePoint[9]);
  for (int i = 10; i <= 14; i++)
  { 
    if (TimePoint[i] < 10)
       saida = saida + "0" + String(TimePoint[i]);
    else saida = saida+ String(TimePoint[i]);
  }
  
	for (int i = 0; i <= 4; i++)
		saida = saida + String(TipAdic[i]);   // Tipos de adiçoes
	saida = saida + String (NumRampas);
  saida = saida + String (NumAdic); 
	int PotInt;
	if (xSemaphoreTake(POTPAN_Mutex,portMAX_DELAY) == true)
  {
	    PotInt = POTPAN;
	    xSemaphoreGive(POTPAN_Mutex);
  }
	String Pot;
	if (PotInt > 9) Pot = String (PotInt);
		else Pot = "0" + String (PotInt);
	saida = saida + Pot;
	return saida;
}

void BluetoothLeConfig (String Param)
{
    Temperatura[0] = Param.substring(0,2).toInt();   // Temperatura das rampas
    Temperatura[1] = Param.substring(2,4).toInt();
    Temperatura[2] = Param.substring(4,6).toInt();
    Temperatura[3] = Param.substring(6,8).toInt();
    Temperatura[4] = Param.substring(8,10).toInt();    

	
    TimePoint[0] = Param.substring(10,12).toInt();   // Tempo das Rampas
    TimePoint[1] = Param.substring(12,14).toInt();
    TimePoint[2] = Param.substring(14,16).toInt();
    TimePoint[3] = Param.substring(16,18).toInt();
    TimePoint[4] = Param.substring(18,20).toInt();
    
	  TimePoint[9] = Param.substring(20,22).toInt();   // tempo de fervura
	
    TimePoint[10] = Param.substring(22,24).toInt();   // Tempo das adiçoes
    TimePoint[11] = Param.substring(24,26).toInt();  
    TimePoint[12] = Param.substring(26,28).toInt();  
    TimePoint[13] = Param.substring(28,30).toInt();   
    TimePoint[14] = Param.substring(30,32).toInt();     

    TipAdic[0] = Param.substring(32,33).toInt();     //Tipos de adiçoes
    TipAdic[1] = Param.substring(33,34).toInt(); 
    TipAdic[2] = Param.substring(34,35).toInt(); 
    TipAdic[3] = Param.substring(35,36).toInt(); 
    TipAdic[4] = Param.substring(36,37).toInt();    

    NumRampas = Param.substring(37,38).toInt();
  	NumAdic = Param.substring(38,39).toInt();
  	String Pot = Param.substring(39,41);
  	int PotInt = Pot.toInt();
    if (xSemaphoreTake(POTPAN_Mutex,portMAX_DELAY) == true)
    {
  	    POTPAN = PotInt;
  	    xSemaphoreGive(POTPAN_Mutex);
    }	
}



String FillStr (String txt, byte len)
{
   String saida = txt;
   while (saida.length() < len)
      saida = saida + " ";
   return saida;
} 

String FormatTimer (long t)
{
    String Saida = "";
    String hstr;
    String mstr;

    int horas = t / 60;
    if (horas < 10)
        hstr = "0"+String (horas);
    else
        hstr = String (horas);
    int minutos = t % 60;
    if (minutos < 10)
        mstr = "0"+ String (minutos);
    else
        mstr = String (minutos);

    Saida =hstr + ":" + mstr;
    return  Saida;
}

String MontaMsgBluetooth (String t1,String t2, float l, long t, byte b, bool a)
{
    String msgl1 =FillStr(t1,16); 
    String msgl2 =FillStr(t2,16); 
    String graustxt = String (l).substring(0, 4)+"º";
    String tempotxt = FormatTimer (t);
    String alarmetxt = String (a);
    String bombatxt =FillStr(String (b),3);
    return msgl1+msgl2+graustxt+tempotxt+bombatxt+alarmetxt;
}

String LeSerialBT ()
{
    String resp = "";
    while (SerialBT.available())
      resp = resp + char (SerialBT.read());
    return resp;
}

void EnviaSerial (String txt)
{
    bool sair = false;
    byte cont = 0;
    while (sair == false)
    {    
        SerialBT.print(txt);
        while (not SerialBT.available() || cont <= 100)
        {
            delay(10);
            ++ cont;
            if (cont == 100)
            {
                sair = true;
                break;
            }   
        }
        
        if (LeSerialBT () == "Ok")
           sair = true;
        delay(10);
    }
}
*/
void loop5(void *z)
{
    String MsgBluetooth =""; 

    while (1)
    {

      delay (100);
    }   

}


// ----------------------------------------------------------------- SEÇÃO DE SETUP ------------------------------------------------------------------------------
void setup() 
{
 
 //   LoadConfig (PANCONFIG,TEMPERATURA, TIMEPOINT, PANPARAM);



 
    Serial.begin(115200);
    Wire.begin();
    
 // Inicializa o LCD 16x2
   
    lcd.init();
    lcd.print("Iniciando...");
    delay (1000);
    lcd.clear();
    lcd.backlight();
    lcd.cursor_off();
    lcd.setCursor(0, 0);
    lcd.createChar(0, grau);

    // Inicializa o Display de 7 segmentos
    display.setBrightness(0x0f); //DEFINE A INTESIDADE LUMINOSA DOS SEGMENTOS DO DISPLAY
    display.showNumberDecEx(0, 0x20, 1, 4, 0);
    
    //Inicializa a classe Take_Semaphore
    init();
    
    //Cria os semaphoros de troca de informações entres as tasks
    ALARME_Mutex = xSemaphoreCreateMutex();
    LEITURA_Mutex = xSemaphoreCreateMutex();
    POTPAN_Mutex = xSemaphoreCreateMutex();
    SETPOINT_Mutex = xSemaphoreCreateMutex();
    ETAPALCD_Mutex = xSemaphoreCreateMutex();
    I_FLAG_Mutex = xSemaphoreCreateMutex();
    TECLA_Mutex = xSemaphoreCreateMutex();
    TIMER_Mutex = xSemaphoreCreateMutex();
    PARAM_Mutex = xSemaphoreCreateMutex();
    CONFIG_Mutex = xSemaphoreCreateMutex();
    ETAPA_Mutex = xSemaphoreCreateMutex();
    


    Serial.begin(115200);
    SerialBT.begin("EasyPan"); //Bluetooth device name

 // Inicializa o teclado I2C
    if (!keyPad.begin()) {
        Serial.print("Cannot connect to I2C.\n");
    }
    keyPad.loadKeyMap(keypad_layout);   

// Cria as Tasks
    xTaskCreatePinnedToCore(loop5, "loop5", 10440, NULL, 2, NULL, 0);//Cria a tarefa "loop5()" com prioridade 2, atribuída ao core 0 - Bluetooth
    xTaskCreatePinnedToCore(loop4, "loop4", 1024, NULL, 2, NULL, 0);//Cria a tarefa "loop4()" com prioridade 2, atribuída ao core 0 - Alarme
    xTaskCreatePinnedToCore(loop3, "loop3", 30440, NULL, 5, NULL, 0);//Cria a tarefa "loop3()" com prioridade 5, atribuída ao core 0 - Interface
    xTaskCreatePinnedToCore(loop2, "loop2", 10440, NULL, 2, NULL, 0);//Cria a tarefa "loop2()" com prioridade 5, atribuída ao core 0 - Mosturação
    xTaskCreatePinnedToCore(loop1, "loop1", 4048, NULL, 5, NULL, 1);//Cria a tarefa "loop1()" com prioridade 5, atribuída ao core 1 - PID

}


void loop() 
{
    delay (10);

}