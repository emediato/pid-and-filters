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
#include <gtk/gtk.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

/**************************************************************************/
// Definicoes
/**************************************************************************/

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


//variaveis da interface
float dado = 10.0;
float P;//kc
float I;//Ti
float D;//Td
float B;//Beta
float SP=0;
float MV=0;
int AUTO=0;

//Encontrar o numero do erro
extern int errno;

//Variáveis da Interface
int handler_id;
GtkWidget *entryP;
GtkWidget *entryI;
GtkWidget *entryD;
GtkWidget *entryBeta;
GtkWidget *entrySP;
GtkWidget *entryMV;
GtkWidget *buttonEnviar = NULL;
GtkWidget *buttonInicializar= NULL;
GtkWidget *modo;
GtkWidget *entryFile;
GtkWidget *entryAmostragem;



//Função que inicializa o o controlador PID e demais processos
static void inicializar (GtkWidget *wid, gpointer win)
{
  GtkWidget *dialog = NULL;
   //Variavel que contem o nome do arquivo de armazenamento de dados
   const char* NomeArq;
   const char* entrada;
   int res;

   entrada = gtk_entry_get_text(GTK_ENTRY (entryP));
   sscanf (entrada, "%f", &Kp);
   entrada = gtk_entry_get_text(GTK_ENTRY (entryI));
   sscanf (entrada, "%f", &Ti);
   entrada = gtk_entry_get_text(GTK_ENTRY (entryD));
   sscanf (entrada, "%f", &Td);
   entrada = gtk_entry_get_text(GTK_ENTRY (entryBeta));
   sscanf (entrada, "%f", &tau_c);
   entrada = gtk_entry_get_text(GTK_ENTRY (entryAmostragem));
   sscanf (entrada, "%f", &TempAms);

   //habilita botao e campos
   gtk_widget_set_sensitive(buttonEnviar,1);
   gtk_widget_set_sensitive(entryMV,1);
   gtk_widget_set_sensitive(modo,1);

   //desabilita botao e campos
   gtk_widget_set_sensitive(buttonInicializar,0);
   gtk_widget_set_sensitive(entryFile,0);
   gtk_widget_set_sensitive(entryAmostragem,0);

   NomeArq = gtk_entry_get_text(GTK_ENTRY (entryFile));

   //Para verificar se o arquivo ja existe, tentamos abri-lo para leitura.
   ArqDados = fopen( NomeArq, "r" );
   if( ArqDados == NULL ){
      //Arquivo nao existe
      ArqDados = fopen( NomeArq, "w+" );
      if( ArqDados == NULL ){
         printf("\n  Erro ao Criar Arquivo %s",NomeArq);
         printf("\n  Significado:%s \n", strerror( errno));
      }
      else printf("\n Arquivo %s criado com sucesso ! \n", NomeArq);
   }
   else{
      GtkWidget *dialog;
      dialog = gtk_message_dialog_new(win,
             GTK_DIALOG_DESTROY_WITH_PARENT,
             GTK_MESSAGE_QUESTION,
             GTK_BUTTONS_YES_NO,
             "Arquivo ja existe. Sobrescrever?");
      gtk_window_set_title(GTK_WINDOW(dialog), "Question");
      gint result = gtk_dialog_run(GTK_DIALOG(dialog));
      if (result==GTK_RESPONSE_NO) {
         printf("\n Os dados NAO estao sendo salvos em arquivo. \n");
      }
      else if(result==GTK_RESPONSE_YES){
         fclose(ArqDados);
         ArqDados = fopen( NomeArq, "w+" );
         if( ArqDados == NULL ){
            printf("\n  Erro ao Criar Arquivo %s",NomeArq);
            printf("\n  Significado:%s \n", strerror( errno));
         }
         else printf("\n Arquivo %s criado com sucesso ! \n", NomeArq);
      }
      gtk_widget_destroy(dialog);
   }

  
  printf("Parâmetros Inicializados\n");
  printf("Tempo de amostragem=%f [ms]\n",TempAms);
  printf("Modo Manual\t");
  printf("MV=%f\n",valor_manual);
  printf("Kp=%f\t",Kp);
  printf("Ti=%f\t",Ti);
  printf("Td=%f\t",Td);
  printf("tau_c=%f\n",tau_c);
  lambda = (tau_c/TempAms*1000)/(1+tau_c/TempAms*1000);

  dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, "Inicializado!");
  gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
  gtk_dialog_run (GTK_DIALOG (dialog));
  gtk_widget_destroy (dialog);
}

//Função para finalizar programa
void terminaPrograma(GtkWidget *widget, gpointer win)
{
   Termina=1; //flag que sinaliza término do programa

   aguarda_finalizacao_das_threads(); //não implementado
   finaliza_tarefas_da_placa_de_aquisição(); //não implementado;
   fclose(ArqDados);        
   printf("Programa terminado.\n");
}


//Função chamada ao clicar no botão Enviar da interface
static void enviar (GtkWidget *wid, gpointer win)
{
  const char* entrada;
  GtkWidget *dialog = NULL;
    printf("Parâmetros atualizados:\n");
   if (AUTO){
      entrada = gtk_entry_get_text(GTK_ENTRY (entrySP));
  }
  else {
      entrada = gtk_entry_get_text(GTK_ENTRY (entryMV));
  }
  sscanf (entrada, "%f", &dado);
  if(dado<0.0 || dado>10.0){
      dialog = gtk_message_dialog_new (GTK_WINDOW (win), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, "Entre com um valor de SP/MV entre 0 e 10!");
      gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
      gtk_dialog_run (GTK_DIALOG (dialog));
      gtk_widget_destroy (dialog);
  }
  else{
      entrada = gtk_entry_get_text(GTK_ENTRY (entryP));
      sscanf (entrada, "%f", &P);
      entrada = gtk_entry_get_text(GTK_ENTRY (entryI));
      sscanf (entrada, "%f", &I);
      entrada = gtk_entry_get_text(GTK_ENTRY (entryD));
      sscanf (entrada, "%f", &D);
      entrada = gtk_entry_get_text(GTK_ENTRY (entryBeta));
      sscanf (entrada, "%f", &B);
  }

   if (AUTO) {
         printf("Modo automatico\t");
         SP=dado;
         printf("SP=%f\n",SP);
   } else{
         printf("Modo automatico\t");
         MV=dado;
         printf("MV=%f\n",MV);
   }

  printf("Kp=%f\t",P);
  printf("Ti=%f\t",I);
  printf("Td=%f\t",D);
  printf("tau_c=%f\n",B);
  
  Kp=P;
  Ti=I;
  Td=D;
  tau_c=B;
  setpoint=SP;
  valor_manual=MV;
  modo_auto=AUTO;
  lambda = (tau_c/TempAms*1000)/(1+tau_c/TempAms*1000);

}


//Botão Auto-manual
void toogle_signal(GtkWidget *widget, gpointer window)
{
  if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(widget))) {
       gtk_widget_set_sensitive(entryMV,0);
       gtk_widget_set_sensitive(entrySP,1);
       gtk_button_set_label(GTK_BUTTON(modo),"Automático");
       AUTO = TRUE;
       printf("Modo automatico\n");
       printf("SP=%f\n",SP);
  } else {
       gtk_widget_set_sensitive(entrySP,0);
       gtk_widget_set_sensitive(entryMV,1);
       gtk_button_set_label(GTK_BUTTON(modo),"Manual");
       AUTO = FALSE;
       printf("Modo manual\n");
       printf("MV=%f\n",MV);
  }
}

//Cria e ativa interface gráfica
static void
activate (GtkApplication* app,
          gpointer        user_data)
{
  GtkWidget *win = NULL;
  GtkWidget *vbox = NULL;
  GtkWidget *grid = NULL;
  GtkWidget *hbox = NULL;
  GtkWidget *labelP = NULL;
  GtkWidget *labelI = NULL;
  GtkWidget *labelD = NULL;
  GtkWidget *labelBeta = NULL;
  GtkWidget *labelSP = NULL;
  GtkWidget *labelMV = NULL;
  GtkWidget *labelMode = NULL;
  GtkWidget *labelAmostragem = NULL;
  GtkWidget *labelFile = NULL;
  GtkWidget *buttonMode = NULL;


  /* Create the main window */
  win = gtk_application_window_new (app);
  gtk_container_set_border_width (GTK_CONTAINER (win), 8);
  gtk_window_set_title (GTK_WINDOW (win), "Controlador PID");
  gtk_window_set_position (GTK_WINDOW (win), GTK_WIN_POS_CENTER);
  gtk_widget_set_size_request(win, 250,400);
  g_signal_connect(G_OBJECT (win), "destroy", G_CALLBACK (terminaPrograma), NULL);

  /* Create a vertical box with buttons */
  vbox = gtk_box_new (GTK_ORIENTATION_VERTICAL, 3);

  //Cria um grid
  grid = gtk_grid_new();
  gtk_container_add(GTK_CONTAINER(vbox), grid);

  gtk_grid_set_row_spacing(GTK_GRID(grid),5);

  labelFile = gtk_label_new("Arquivo");
  labelAmostragem = gtk_label_new("Ts (ms)");
  labelP = gtk_label_new("Kp");
  labelI = gtk_label_new("Ti (s)");
  labelD = gtk_label_new("Td (s)");
  labelBeta = gtk_label_new("tau_c (s)");
  labelSP = gtk_label_new("SP");
  labelMV = gtk_label_new("MV");
  labelMode = gtk_label_new("Modo");

  //Coloca texto na primeira coluna
  gtk_grid_attach(GTK_GRID(grid), labelFile, 0, 0, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelAmostragem, 0, 1, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelP, 0, 2, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelI, 0, 3, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelD, 0, 4, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelBeta, 0, 5, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelMode, 0, 6, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelSP, 0, 7, 1, 1);
  gtk_grid_attach(GTK_GRID(grid), labelMV, 0, 8, 1, 1);

  entryFile = gtk_entry_new();
  entryAmostragem = gtk_entry_new();
  entryP = gtk_entry_new();
  entryI = gtk_entry_new();
  entryD = gtk_entry_new();
  entryBeta = gtk_entry_new();
  entrySP = gtk_entry_new();
  entryMV = gtk_entry_new();

  //Coloca caixa de entrada de texto na segunda coluna
  gtk_grid_attach(GTK_GRID(grid), entryFile, 1, 0, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryAmostragem, 1, 1, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryP, 1, 2, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryI, 1, 3, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryD, 1, 4, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryBeta, 1, 5, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entrySP, 1, 7, 2, 1);
  gtk_grid_attach(GTK_GRID(grid), entryMV, 1, 8, 2, 1);

    modo=gtk_toggle_button_new_with_label ("Manual");
    gtk_grid_attach(GTK_GRID(grid), modo, 1, 6, 1,1);

  g_signal_connect(G_OBJECT(modo), "toggled",G_CALLBACK(toogle_signal),(gpointer)win);

  gtk_widget_show(grid);
  //desabilita campos
  gtk_widget_set_sensitive(entrySP,0);
  gtk_widget_set_sensitive(entryMV,0);
//  gtk_widget_set_sensitive(check,0);
  gtk_widget_set_sensitive(modo,0);

  hbox = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 2);

  buttonInicializar = gtk_button_new_with_label ("Inicializar");
  gtk_widget_set_margin_start(buttonInicializar,50);
  g_signal_connect (G_OBJECT (buttonInicializar), "clicked", G_CALLBACK (inicializar),(gpointer) win);
  gtk_box_pack_start (GTK_BOX (hbox), buttonInicializar, FALSE, TRUE, 0);

  buttonEnviar = gtk_button_new_with_label ("Enviar");
  g_signal_connect( G_OBJECT(buttonEnviar), "clicked", G_CALLBACK (enviar), (gpointer) win);
  gtk_box_pack_start (GTK_BOX (hbox), buttonEnviar, FALSE, TRUE, 0);
  gtk_widget_set_sensitive(buttonEnviar,0);

  gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 3);

  gtk_container_add (GTK_CONTAINER (win), vbox);
  /* Enter the main loop */
  gtk_widget_show_all (win);

}//activate 

int main (int argc, char *argv[])
{
    GtkApplication *app;
    int status;

  app = gtk_application_new ("controlador.app", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return 0;
}
