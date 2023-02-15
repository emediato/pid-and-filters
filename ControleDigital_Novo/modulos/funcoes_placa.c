
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <NIDAQmx.h>


float TempAms;   //tempo de amostragem em milesegundos
long  Ts;        //tempo de amostragem em nanosegundos

//Variáveis da placa de aquisição
int32       error=0;
char        errBuff[2048]={'\0'};
TaskHandle  AItaskHandle=0;
TaskHandle  AOtaskHandle=0;

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

//Função para inicializar driver da placa de aquisição
static int Inicializa_placa(void)
{
	double SamplesPerSecond;
	
	SamplesPerSecond = 1000/TempAms;
	Ts = (long) (TempAms*1000000);

	DAQmxErrChk (DAQmxCreateTask("", &AItaskHandle)); //cria tarefa de entrada analógica
	DAQmxErrChk (DAQmxCreateTask("", &AOtaskHandle)); //cria tarefa de saída analógica

    //configura canal de entrada
	DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, "Dev1/ai0", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
    //configura canal de saída
	DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0", "", -10.0, 10.0, DAQmx_Val_Volts, NULL));

    //configura timers para tarefas de leitura e escrita
	DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", SamplesPerSecond, 	DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,1));//DAQmx_Val_ContSamps or DAQmx_Val_HWTimedSinglePoint
	DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "",SamplesPerSecond, DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,1));//DAQmx_Val_HWTimedSinglePoint or DAQmx_Val_ContSamps
	

        DAQmxSetRealTimeWaitForNextSampClkWaitMode(AItaskHandle,DAQmx_Val_WaitForInterrupt); //permitirá que a thread de tempo real durma enquanto aguarda o próximo interrupt
        DAQmxSetRealTimeConvLateErrorsToWarnings(AItaskHandle,1);
        
        DAQmxSetReadWaitMode(AItaskHandle,DAQmx_Val_Poll);//evita que tarefa durma enquanto aguarda a leitura, o que poderia ceder o controle para um processo de menor prioridade

	// Inicia tarefas
	DAQmxErrChk(DAQmxStartTask(AItaskHandle));
	DAQmxErrChk(DAQmxStartTask(AOtaskHandle));

out:
        return 0;

Error:
       if( DAQmxFailed(error) )
             DAQmxGetExtendedErrorInfo(errBuff,2048);
       if( AItaskHandle!=0 )  {
              DAQmxStopTask(AItaskHandle);
              DAQmxClearTask(AItaskHandle);
       }
       if( DAQmxFailed(error) )
              printf("DAQmx Error: %s\n",errBuff);
       return 0;
	
}// Inicializa_placa

static float64 le_placa(void){
        float64 read_data;
       	DAQmxErrChk(DAQmxReadAnalogScalarF64(AItaskHandle, 10.0,&read_data,NULL)); //leitura da placa no canal 0
out:
        return read_data;

Error:
       if( DAQmxFailed(error) )
             DAQmxGetExtendedErrorInfo(errBuff,2048);
       if( AItaskHandle!=0 )  {
              DAQmxStopTask(AItaskHandle);
              DAQmxClearTask(AItaskHandle);
       }
       if( DAQmxFailed(error) )
              printf("DAQmx Error while reading: %s\n",errBuff);
       return -1;
 }
 
void escreve_placa(float64 write_data){
         DAQmxErrChk(DAQmxWriteAnalogScalarF64(AOtaskHandle, 1,0.0,write_data,NULL));//escrita na placa no canal 0
out:
        return;

Error:
       if( DAQmxFailed(error) )
             DAQmxGetExtendedErrorInfo(errBuff,2048);
       if( AOtaskHandle!=0 )  {
              DAQmxStopTask(AOtaskHandle);
              DAQmxClearTask(AOtaskHandle);
       }
       if( DAQmxFailed(error) )
              printf("DAQmx Error while writing: %s\n",errBuff);
       return;
}

void finaliza_placa(void){
   DAQmxStopTask(AItaskHandle);
   DAQmxClearTask(AItaskHandle);
   DAQmxStopTask(AOtaskHandle);
   DAQmxClearTask(AOtaskHandle);
}

static void wait_for_next_sampling_time(void)
{
	int islate;
        DAQmxWaitForNextSampleClock(AItaskHandle,15.0,&islate); //usar se hardware-timed
}


