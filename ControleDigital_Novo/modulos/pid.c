//Controlador PID usando o driver NIDAQmx
// Para compilar:
//gcc `pkg-config --cflags gtk+-3.0` pid.c -o pid `pkg-config --libs gtk+-3.0` /usr/lib/x86_64-linux-gnu/libnidaqmx.so.22.5.0 -pthread -lm
// Para acompanhar os processos e suas prioridades:
// ps -eLo pid,class,rtprio,ni,comm | grep -i "pid_app"
// Para acompanhar interrupções relevantes (nipalk para o driver,rtc0 para o relógio):
// ps -eLo pid,class,rtprio,ni,comm | grep -i "irq"
// Para mudar prioridade das interrupções:
// chrt -f -p 90 pid #onde pid é o process id 

/**************************************************************************/
// Bibliotecas
/**************************************************************************/

#include <stdlib.h>
#include <pthread.h>
#include <limits.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <math.h>

/**************************************************************************/
// Definicoes
/**************************************************************************/

#define TAM_BUFFER 100      //tamanho do buffer de dados do controlador
#define MULTIPLICADOR 10    //fator de amostragem para cópia do buffer em arquivo
#define UMAX 10.0           //limite superior em Volts da saída
#define UMIN 0.0            //limite inferior em Volts da saída

/**************************************************************************/
// Variaveis globais
/**************************************************************************/

// flag para sinalizar fim de programa
int Termina = 0;

//Arquivo:
FILE *ArqDados;

float TempAms;   //tempo de amostragem em milesegundos
long  Ts;        //tempo de amostragem em nanosegundos

//Variáveis do buffer circular
long cicloInicio[TAM_BUFFER];
long cicloFim[TAM_BUFFER];
double buffer_leitura[TAM_BUFFER];
double buffer_escrita[TAM_BUFFER];
double buffer_setpoint[TAM_BUFFER];
unsigned int buffer_ModoManual[TAM_BUFFER];
double buffer_leitura_filtrado[TAM_BUFFER];

long copia_cicloInicio[MULTIPLICADOR*2];
long copia_cicloFim[MULTIPLICADOR*2];
double copia_buffer_leitura[MULTIPLICADOR*2];
double copia_buffer_escrita[MULTIPLICADOR*2];
double copia_buffer_setpoint[MULTIPLICADOR*2];
unsigned int copia_buffer_ModoManual[MULTIPLICADOR*2];
double copia_buffer_leitura_filtrado[MULTIPLICADOR*2];

// Indexacao do buffer global
unsigned int read_index = 0;//posição de leitura
unsigned int count=0;        //posição de escrita

//Threads
pthread_t thread_escrita_arquivo;
pthread_t thread_controlador;

struct period_info {
        struct timespec next_period;
        long period_ns;
};

//Variáveis do controlador
double SomaErro=0;
double erro_anterior=0;

//Variáveis do filtro anti-spike
double Deltamed=0;
double D0=0;
double Dm1=0;
double Dm2=0;
double yanterior=0;
int SpikeCount=0;

//Variáveis do filtro passa-baixas
double yf=0;
double lambda =0;

//Parametros do controlador
float Kp = 1.0;
float Ti = 1.0;
float Td = 0.1;
float tau_c = 100;
float setpoint=0.0;
float valor_manual=0.0;
int modo_auto=0;


//Função para incrementar o target usado para a função clock_nanosleep
static void inc_period(struct period_info *pinfo)
{
        pinfo->next_period.tv_nsec += pinfo->period_ns;

        while (pinfo->next_period.tv_nsec >= 1000000000) {
                /* timespec nsec overflow */
                pinfo->next_period.tv_sec++;
                pinfo->next_period.tv_nsec -= 1000000000;
        }
}

//Função para inicializar o target usado para a função clock_nanosleep
static void periodic_task_init(struct period_info *pinfo,long T_s)
{
        /* for simplicity, hardcoding a 1ms period */
        pinfo->period_ns = T_s;
        clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

//Função que realiza o filtro passa-baixas
static double filtro_passa_baixas(double y){
yf = lambda*yf + (1-lambda) *y;
return yf;
}

//Função que realiza o filtro anti-spike
static double filtroAntiSpike(double y){

double yout,Dmed,d1,d2,dmed;

D0=y-yanterior; //incremento atual
//Calcula mediana e MAD dos últimos 3 incrementos
if (D0>Dm2){
   if (D0<Dm1)
   {
     Dmed=D0;
     d1=fabs(Dm1-D0);
     d2=fabs(Dm2-D0);
     dmed = (d1>d2 ? d2 : d1);
   }else if (Dm1>Dm2){
     Dmed=Dm1;
     d1=fabs(Dm1-D0);
     d2=fabs(Dm2-Dm1);
     dmed = (d1>d2 ? d2 : d1);
   }else
   {
     Dmed=Dm2;
     d1=fabs(Dm2-D0);
     d2=fabs(Dm1-Dm2);
     dmed = (d1>d2 ? d2 : d1);
   }
}  else if (D0>Dm1){
     Dmed=D0; 
     d1=fabs(Dm1-D0);
     d2=fabs(Dm2-D0);
     dmed = (d1>d2 ? d2 : d1);
   } else if (Dm1<Dm2){
     Dmed=Dm1;
     d1=fabs(Dm1-D0);
     d2=fabs(Dm2-Dm1);
     dmed = (d1>d2 ? d2 : d1);
   } else{
     Dmed=Dm2;
     d1=fabs(Dm2-D0);
     d2=fabs(Dm1-Dm2);
     dmed = (d1>d2 ? d2 : d1);
   }

Deltamed+= 0.1*(dmed-Deltamed); //filtra o Median Absolute Deviation

yout = y;
//Identifica spikes/outliers e substitui pela variação mediana
if (fabs(D0-Dmed)>(3*1.4826*Deltamed))
{ 
yout=yanterior+Dmed;
SpikeCount++;
}

Dm2=Dm1;
Dm1=D0;
yanterior = yout;
return yout;
}// Filtro anti-spike

//Função que realiza o controlador PID
static double calcula_sinal_de_controle(double y)
{
double u;
double DeltaErro;


u=(setpoint-y);
SomaErro+= u*TempAms/1000/Ti;
DeltaErro = (u - erro_anterior)*Td/TempAms*1000;
erro_anterior = u;
u=Kp*(u+SomaErro+DeltaErro);

if (!modo_auto){
u=valor_manual;
}

if (u>UMAX) u=UMAX;
if (u<UMIN) u=UMIN;

return u;
}//calcula_sinal_de_controle

//Tarefa de tempo real do controlador
static void executa_controlador()
{
        /* Do RT stuff here. */
        int i;
        float64 read_data;
        float64 write_data;

       	read_data = FUNÇÃO_PARA_LER_PLACA();  //essa função ainda não existe!
        buffer_leitura[count] = read_data;
	    write_data = calcula_sinal_de_controle(read_data);             
	    FUNÇÃO_PARA_ESCREVER_NA_PLACA(write_data); //essa função ainda não existe!
        buffer_escrita[count] = write_data;
        buffer_ModoManual[count] = modo_auto;
        buffer_setpoint[count] = setpoint;
        buffer_leitura_filtrado[count] = read_data;
   	        
}//executa_controlador


//Thread de tempo real para execução do controlador
void *Thread_Controlador(void *data)
{
        struct timespec tempo;
        struct timespec tempo2;
        int ret;
 
         while (!Termina) {
                ret=clock_gettime(CLOCK_MONOTONIC, &tempo);
                
                executa_controlador();
                
                cicloInicio[count]=tempo.tv_sec*1000000+tempo.tv_nsec/1000; //tempo em usec
                ret=clock_gettime(CLOCK_MONOTONIC, &tempo2);
                cicloFim[count]=tempo2.tv_sec*1000000+tempo2.tv_nsec/1000; //tempo em usec
                count++;//incrementa posição de escrita no buffer
                if (count == TAM_BUFFER) count = 0; //buffer circular
                               
            	if (count==read_index){
          	    read_index++;
        	     if (read_index==TAM_BUFFER) read_index=0;
        	     printf("Atenção: Entradas do buffer descartadas!\n");
        	}
                aguardar_proxima_amostra(); //ainda não implementado
        }
        return NULL;
        
}//thread_controlador

//Thread de tempo real para escrita dos dados históricos em arquivo
void *Thread_Escrita_Arquivo(){

   // Índices buffer auxiliar e para cópia de dados para arquivo
   int k = 0;
   int cont = 0;

   struct period_info pinfo;  //variável do timer
 
   periodic_task_init(&pinfo,Ts*MULTIPLICADOR);  //inicializa timer

 
   fprintf(ArqDados,"Leitura Leitura_filtrada Escrita ModoOperacao ValorDesejado CicloInicio CicloFim\n");
   while(!Termina){
      // Inicializa variaveis
      cont = 0;
      k = 0;

      while(read_index!=count){
         //Escreve os valores de leitura no buffer auxiliar
         copia_cicloInicio[k]=cicloInicio[read_index];
    	 copia_cicloFim[k]=cicloFim[read_index];
	     copia_buffer_leitura[k]=buffer_leitura[read_index];
    	 copia_buffer_escrita[k]=buffer_escrita[read_index];
	     copia_buffer_setpoint[k]=buffer_setpoint[read_index];
    	 copia_buffer_ModoManual[k]=buffer_ModoManual[read_index];
	     copia_buffer_leitura_filtrado[k]=buffer_leitura_filtrado[read_index];

         read_index++; //atualiza posição de leitura
         if (read_index == TAM_BUFFER) read_index = 0;  //Buffer Circular
         k++;
         if (k==(2*MULTIPLICADOR)) break;
      }//while

      // Transfere dados do buffer auxiliar para arquivo
      while(cont != k){
         fprintf(ArqDados,"%f %f %f %u %f %ld %ld\n",  copia_buffer_leitura[cont], copia_buffer_leitura_filtrado[cont], copia_buffer_escrita[cont], copia_buffer_ModoManual[cont], copia_buffer_setpoint[cont],copia_cicloInicio[cont],copia_cicloFim[cont]);
         cont ++;
      }// while
      //DORME
      inc_period(&pinfo);        
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(pinfo.next_period), NULL);
   }//While                                           
   printf("Fim Thread_Escrita_Arquivo\n");
   return NULL;
}//  Thread_Escrita_Arquivo


//Função para iniciar threads de tempo real
static void inicializa_threads(void){
    struct sched_param param; //parâmetro que contém prioridade da thread
    pthread_attr_t attr_controlador; //atributos das threads
    pthread_attr_t attr_escrita_arquivo;

    int ret;
    int j;

 	struct rlimit rlim;

	pthread_mutexattr_t mattr1;//atributos dos mutexes
	pthread_mutexattr_t mattr2;

    //Define tempo máximo que um processo de tempo real pode ocupar o processador
    //Evita que o programa congele o computador
	getrlimit(RLIMIT_RTTIME,&rlim);
	rlim.rlim_cur=10000000;//10 segundos
	setrlimit(RLIMIT_RTTIME,&rlim);
	
	//Definie herança de prioridade para os mutexes
	pthread_mutexattr_init(&mattr1);
	pthread_mutexattr_init(&mattr2);
	pthread_mutexattr_setprotocol(&mattr1,PTHREAD_PRIO_INHERIT);
	pthread_mutexattr_setprotocol(&mattr2,PTHREAD_PRIO_INHERIT);
	pthread_mutexattr_setrobust(&mattr1,PTHREAD_MUTEX_ROBUST_NP);
	pthread_mutexattr_setrobust(&mattr2,PTHREAD_MUTEX_ROBUST_NP);
	pthread_mutex_init(&AcessaBuffer,&mattr1);
	pthread_mutex_init(&AcessaDadosEntrada,&mattr2);
		
      /* Lock memory */
      if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                printf("mlockall failed: %m\n");
                exit(-2);
        }
      

        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr_controlador);
        if (ret) {
                printf("init pthread attributes failed\n");
                goto out;
        }
        
        ret = pthread_attr_init(&attr_escrita_arquivo);
        if (ret) {
                printf("init pthread attributes failed\n");
                goto out;
        }


        /* Set a specific stack size  */
        ret = pthread_attr_setstacksize(&attr_controlador, PTHREAD_STACK_MIN);
        if (ret) {
            printf("pthread setstacksize failed\n");
            goto out;
        }
        ret = pthread_attr_setstacksize(&attr_escrita_arquivo, PTHREAD_STACK_MIN);
        if (ret) {
            printf("pthread setstacksize failed\n");
            goto out;
        }


        /* Create a pthread with specified attributes */
        ret = pthread_create(&thread_controlador, &attr_controlador, Thread_Controlador, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
        ret = pthread_create(&thread_escrita_arquivo, &attr_escrita_arquivo, Thread_Escrita_Arquivo, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
out:
        return;
}//inicializa threads

void finaliza_threads(void){
    int ret;
   /* Join the thread and wait until it is done */
   ret = pthread_join(thread_controlador, NULL);
   if (ret)
      printf("join pthread failed: %m\n");
   ret = pthread_join(thread_escrita_arquivo, NULL);
   if (ret)
      printf("join pthread failed: %m\n");
}



