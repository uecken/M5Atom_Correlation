/*Press button to record,released button to playback*/
#include <Arduino.h>
#include <driver/i2s.h>
#include <M5Atom.h>

#define CONFIG_I2S_BCK_PIN 19
#define CONFIG_I2S_LRCK_PIN 33
#define CONFIG_I2S_DATA_PIN 22
#define CONFIG_I2S_DATA_IN_PIN 23

#define SPEAKER_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1
#define DATA_SIZE 1024

//uint8_t microphonedata0[1024 * 100]; //約3秒間の音声入力信号を格納 (1024*100)/(16000*2Byte)
uint8_t microphonedata0[1024 * 50]; //約1.5秒間の音声入力信号を格納 (1024*100)/(16000*2Byte)
uint8_t microphonedata_prev[1024 * 50];
//uint8_t cross_cor[1024 * 99];
int data_offset = 0;

void InitI2SSpeakerOrMic(int mode)
{
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(SPEAKER_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
    };
    if (mode == MODE_MIC)
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }
    else
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }

    err += i2s_driver_install(SPEAKER_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;

    //Serial.println("Init i2s_set_pin");
    err += i2s_set_pin(SPEAKER_I2S_NUMBER, &tx_pin_config);
    //Serial.println("Init i2s_set_clk");
    err += i2s_set_clk(SPEAKER_I2S_NUMBER, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);//16bitで良いのか？
}

void setup() {
    M5.begin(true, false, true);
    Serial.begin(115200);
    M5.dis.drawpix(0, CRGB(128, 128, 0));
    delay(2000);
}

#define XCORR_THRESH 0.8
#define SERIES_X_SIZE 51200
#define SERIES_Y_SIZE 51200
	double *cross_correlation(int *series_x, int *series_y){	

		double sum_x = 0.0;
		double sum_y = 0.0;
		double mean_x = 0.0;
		double mean_y = 0.0;
		int delay_max = SERIES_X_SIZE;
		double *cross_correlate = (double*)malloc(((delay_max*2)+1)*sizeof(double));

		//Calculate the sum of each series
			int z;
			for(z = 0; z<SERIES_X_SIZE; z++){
				sum_x += series_x[z];
				sum_y += series_y[z];
			}

		//Calculate mean of each series.
			mean_x = sum_x/SERIES_X_SIZE;
			mean_y = sum_y/SERIES_Y_SIZE;

		//Compute cross-correlate coefficients
			double term_1, term_2, numerator;
			double sqr_series_x = 0.0;
			double sqr_series_y = 0.0;
			int delay, i;
			for(delay = -delay_max; delay<=delay_max; delay++){

				numerator = 0;
				sqr_series_x = 0;
				sqr_series_y = 0;

				for(i =0; i<SERIES_X_SIZE; i++){

					//Assume cofficients with out of bound indicies have x[i]=0, y[i]=0
					int j = i - delay;
					if (j < 0 || j >= SERIES_X_SIZE){
						term_1 = (series_x[i]-mean_x);
						term_2 = (-1*mean_y);	
						numerator += term_1*term_2;
						sqr_series_x += term_1*term_1;
						sqr_series_y += term_2*term_2;			
					}
					else{
						term_1 = (series_x[i]-mean_x);
						term_2 = (series_y[j]-mean_y);	
						numerator += term_1*term_2;
						sqr_series_x += term_1*term_1;
						sqr_series_y += term_2*term_2;
					}

				}

				cross_correlate[delay+delay_max] = numerator/sqrt(sqr_series_x*sqr_series_y);
				printf("delay:%d correlate:%f\n",delay, cross_correlate[delay+delay_max]);
			}

		return cross_correlate;
	}


void loop() {
    if (M5.Btn.isPressed())
    {
        data_offset = 0;
        InitI2SSpeakerOrMic(MODE_MIC);
        size_t byte_read;
        
        //wait for initialization at least 0.5seconds
        delay(500);
        M5.dis.drawpix(0, CRGB(0, 0, 128));

        while (1)
        {
            i2s_read(SPEAKER_I2S_NUMBER, (char *)(microphonedata0 + data_offset), DATA_SIZE, &byte_read, (100 / portTICK_RATE_MS));
            data_offset += 1024;
            M5.update();
            if (M5.Btn.isReleased()){
                break;
            }
            //delay(60);
        }
        size_t bytes_written;
        InitI2SSpeakerOrMic(MODE_SPK);
        i2s_write(SPEAKER_I2S_NUMBER, microphonedata0, data_offset, &bytes_written, portMAX_DELAY);
        
        long cor = 0;
        for(int i=0; i<sizeof(microphonedata_prev);i++){
          cor = cor + microphonedata0[i] * microphonedata_prev[i];
        }
        //0~23000sampleは雑音重畳のため以降のデータで録音・相関等行う
        for(int i=0; i<30000;i=i+1000){ 
          Serial.printf("i:%d, sig1:%d sig2:%d \n",i ,microphonedata0[i], microphonedata_prev[i]);
        }
        //cor = cor / float(sizeof(microphonedata_prev));
        Serial.printf("cor=%ld",cor);

        //Compute the cross correlation coefficients
        double *coefficients = cross_correlation(microphonedata0,microphonedata_prev);

        //Determine if a correlation coefficient >XCORR_THRESH exists.
        int i;
        for(i=0; i<SERIES_X_SIZE*2; i++){
          if(fabs(coefficients[i]) > XCORR_THRESH){
            return MATCH;
          }
        }
        

        memset(microphonedata_prev,0,sizeof(microphonedata_prev));
        memcpy(microphonedata_prev,microphonedata0,sizeof(microphonedata0));

        M5.dis.drawpix(0, CRGB(0, 128, 0));
    }
    M5.update();
}