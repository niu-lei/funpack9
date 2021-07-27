#include "algo_builder.h"

float DummyNodeFloat[3];
const char Identification_String[] = "ID_STRING:sensortitlebox.xml,On-line";

float Absolute_Value_1_out[1];
float Absolute_Value_2_out[1];
float Absolute_Value_5_out[1];
float Acceleration_g_1_data[3];
float Buffer_Out_1_out1[1];
float Buffer_Out_2_out1[1];
float Buffer_Out_3_out1[1];
float Buffer_Out_4_out1[1];
float Buffer_Out_5_out1[1];
float Buffer_Out_6_out1[1];
float Buffer_Out_7_out1[1];
float Buffer_Out_8_out1[1];
float Buffer_Out_9_out1[1];
float Constant_Float_1_out[1];
float Constant_Float_2_out[1];
float Constant_Float_3_out[1];
float Constant_Float_4_out[1];
float Constant_Float_5_out[1];
float Constant_Float_6_out[1];
float Constant_Float_7_out[1];
float Constant_Float_8_out[1];
float Constant_Float_9_out[1];
float Demux_Float_1_out3[1];
float Demux_Float_1_out2[1];
float Demux_Float_1_out1[1];
float Demux_Float_2_out3[1];
float Demux_Float_2_out1[1];
float Demux_Float_2_out2[1];
float Demux_Float_3_out2[1];
float Demux_Float_3_out1[1];
float Demux_Float_3_out3[1];
float Humidity_percent_1_data[1];
float Mux_Float_1_out[2];
float Mux_Float_2_out[2];
float Mux_Float_3_out[2];
float Mux_Float_4_out[6];
float Mux_Float_5_out[3];
float Mux_Float_6_out[2];
float Mux_Float_7_out[2];
float Mux_Float_8_out[2];
float Mux_Float_9_out[2];
float Mux_Float_10_out[2];
float Mux_Float_11_out[2];
float Pressure_hPa_1_data[1];
float Quaternions_9X_1_data[4];
float Temperature_C_1_data[1];
float Tilt_Sensing_1_data[3];
float Tilt_Sensing_2_data[3];
int32_t And_1_out[1];
int32_t And_2_out[1];
int32_t And_3_out[1];
int32_t And_5_out[1];
int32_t And_6_out[1];
int32_t Greater_than_1_out[1];
int32_t Greater_than_2_out[1];
int32_t Less_than_1_out[1];
int32_t Less_than_2_out[1];
int32_t Less_than_3_out[1];
int32_t Less_than_4_out[1];
int32_t Less_than_5_out[1];
void *Sensor_Hub_1_out;


void Demux_3_float(float *in, float *out1, float *out2, float *out3)
{
	*out1 = in[0];
	*out2 = in[1];
	*out3 = in[2];
}

void Mux_2_float(float *in1, float *in2, float *out)
{
	out[0] = *in1;
	out[1] = *in2;
}

void Mux_6_float(float *in1, float *in2, float *in3, float *in4, float *in5, float *in6, float *out)
{
	out[0] = *in1;
	out[1] = *in2;
	out[2] = *in3;
	out[3] = *in4;
	out[4] = *in5;
	out[5] = *in6;
}

void Mux_3_float(float *in1, float *in2, float *in3, float *out)
{
	out[0] = *in1;
	out[1] = *in2;
	out[2] = *in3;
}

sDISPLAY_INFO display_info_list[] = {
{INFO_TYPE_FUSION,1,4,VAR_TYPE_FLOAT,0,"fusion|Sensor\nFusion|Nucleo Board",0},
{INFO_TYPE_GRAPH,2,3,VAR_TYPE_FLOAT,16,"graph|g-graph||1|19|Waveform 1|Waveform 2|Waveform 3||||0",0},
{INFO_TYPE_FLOAT,3,6,VAR_TYPE_FLOAT,28,"float|qingjiao|pitch||roll||gravity inclination||theta||psi||phi angles|||||",0},
{INFO_TYPE_FLOAT,4,3,VAR_TYPE_FLOAT,52,"float|Values|Temp||Humi||Perss|||||||||||",0},
{INFO_TYPE_GRAPH,5,1,VAR_TYPE_FLOAT,64,"graph|press||1|19|Waveform 1||||||0",0},
{INFO_TYPE_GRAPH,6,1,VAR_TYPE_FLOAT,68,"graph|humi||1|19|Waveform 1||||||0",0},
{INFO_TYPE_GRAPH,7,1,VAR_TYPE_FLOAT,72,"graph|temp||1|19|Waveform 1||||||0",0},
{0,0,0,0,0,0,0}};


void AB_Init(void)
{
	Constant_Float_2_out[0] = 0;
	Sensor_Hub_Init(0, 100, 1, 1);
	Accelero_Init();
	Constant_Float_1_out[0] = 0.65;
	MotionTL_Init(0);
	MotionTL_Init(1);
	Magneto_Init();
	MotionFX_Init();
	Constant_Float_5_out[0] = -0.65;
	Constant_Float_8_out[0] = 0.3;
	Constant_Float_4_out[0] = 0;
	Temperature_Init();
	Constant_Float_6_out[0] = -0.65;
	Constant_Float_3_out[0] = 0;
	Humidity_Init();
	Constant_Float_7_out[0] = 0.65;
	Constant_Float_9_out[0] = 0;
	Pressure_Init();
	Message_Length = 76;
}

void AB_Handler(void)
{
	Sensor_Hub_Handler(&Sensor_Hub_1_out);
	Accelero_Sensor_GetData(Sensor_Hub_1_out, Acceleration_g_1_data);
	Demux_3_float(Acceleration_g_1_data, Demux_Float_1_out1, Demux_Float_1_out2, Demux_Float_1_out3);
	if (Demux_Float_1_out3[0] > Constant_Float_1_out[0]) Greater_than_1_out[0] = 1; else Greater_than_1_out[0] = 0;
	if (Greater_than_1_out[0]) {TiltSensing_GetData(Sensor_Hub_1_out, Tilt_Sensing_1_data);}
	Demux_3_float(Tilt_Sensing_1_data, Demux_Float_2_out1, Demux_Float_2_out2, Demux_Float_2_out3);
	Mux_2_float(Constant_Float_2_out, Demux_Float_2_out1, Mux_Float_1_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_1_out1[0] = Mux_Float_1_out[Greater_than_1_out[0]];
	Mux_2_float(Constant_Float_2_out, Demux_Float_2_out2, Mux_Float_2_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_2_out1[0] = Mux_Float_2_out[Greater_than_1_out[0]];
	Mux_2_float(Constant_Float_2_out, Demux_Float_2_out3, Mux_Float_3_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_3_out1[0] = Mux_Float_3_out[Greater_than_1_out[0]];
	if (Greater_than_1_out[0]) {TiltSensing_GetData(Sensor_Hub_1_out, Tilt_Sensing_2_data);}
	Demux_3_float(Tilt_Sensing_2_data, Demux_Float_3_out1, Demux_Float_3_out2, Demux_Float_3_out3);
	Mux_2_float(Constant_Float_2_out, Demux_Float_3_out2, Mux_Float_9_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_7_out1[0] = Mux_Float_9_out[Greater_than_1_out[0]];
	Mux_2_float(Constant_Float_2_out, Demux_Float_3_out1, Mux_Float_11_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_8_out1[0] = Mux_Float_11_out[Greater_than_1_out[0]];
	Mux_2_float(Constant_Float_2_out, Demux_Float_3_out3, Mux_Float_10_out);
	if (Greater_than_1_out[0] < 2) Buffer_Out_9_out1[0] = Mux_Float_10_out[Greater_than_1_out[0]];
	Mux_6_float(Buffer_Out_1_out1, Buffer_Out_2_out1, Buffer_Out_3_out1, Buffer_Out_8_out1, Buffer_Out_7_out1, Buffer_Out_9_out1, Mux_Float_4_out);
	Magneto_Sensor_GetData(Sensor_Hub_1_out, DummyNodeFloat);
	if (Greater_than_1_out[0]) {Quaternions9X_GetData(Sensor_Hub_1_out, Quaternions_9X_1_data);}
	if (Demux_Float_1_out2[0] < Constant_Float_5_out[0]) Less_than_3_out[0] = 1; else Less_than_3_out[0] = 0;
	Absolute_Value_5_out[0] = fabs(Demux_Float_1_out3[0]);
	if (Absolute_Value_5_out[0] < Constant_Float_8_out[0]) Less_than_4_out[0] = 1; else Less_than_4_out[0] = 0;
	Absolute_Value_1_out[0] = fabs(Demux_Float_1_out1[0]);
	if (Absolute_Value_1_out[0] < Constant_Float_8_out[0]) Less_than_1_out[0] = 1; else Less_than_1_out[0] = 0;
	And_5_out[0] = Less_than_1_out[0] && Less_than_4_out[0];
	And_1_out[0] = Less_than_3_out[0] && And_5_out[0];
	if (And_1_out[0]) {Temperature_Sensor_GetData(Sensor_Hub_1_out, Temperature_C_1_data);}
	Mux_2_float(Constant_Float_4_out, Temperature_C_1_data, Mux_Float_7_out);
	if (And_1_out[0] < 2) Buffer_Out_5_out1[0] = Mux_Float_7_out[And_1_out[0]];
	if (Demux_Float_1_out3[0] < Constant_Float_6_out[0]) Less_than_5_out[0] = 1; else Less_than_5_out[0] = 0;
	Absolute_Value_2_out[0] = fabs(Demux_Float_1_out2[0]);
	if (Absolute_Value_2_out[0] < Constant_Float_8_out[0]) Less_than_2_out[0] = 1; else Less_than_2_out[0] = 0;
	And_6_out[0] = Less_than_1_out[0] && Less_than_2_out[0];
	And_2_out[0] = Less_than_5_out[0] && And_6_out[0];
	if (And_2_out[0]) {Humidity_Sensor_GetData(Sensor_Hub_1_out, Humidity_percent_1_data);}
	Mux_2_float(Constant_Float_3_out, Humidity_percent_1_data, Mux_Float_6_out);
	if (And_2_out[0] < 2) Buffer_Out_4_out1[0] = Mux_Float_6_out[And_2_out[0]];
	if (Demux_Float_1_out2[0] > Constant_Float_7_out[0]) Greater_than_2_out[0] = 1; else Greater_than_2_out[0] = 0;
	And_3_out[0] = Greater_than_2_out[0] && And_5_out[0];
	if (And_3_out[0]) {Pressure_Sensor_GetData(Sensor_Hub_1_out, Pressure_hPa_1_data);}
	Mux_2_float(Constant_Float_9_out, Pressure_hPa_1_data, Mux_Float_8_out);
	if (And_3_out[0] < 2) Buffer_Out_6_out1[0] = Mux_Float_8_out[And_3_out[0]];
	Mux_3_float(Buffer_Out_5_out1, Buffer_Out_4_out1, Buffer_Out_6_out1, Mux_Float_5_out);
	Display_Update(Quaternions_9X_1_data, &display_info_list[0]);
	Display_Update(Acceleration_g_1_data, &display_info_list[1]);
	Display_Update(Mux_Float_4_out, &display_info_list[2]);
	Display_Update(Mux_Float_5_out, &display_info_list[3]);
	Display_Update(Buffer_Out_6_out1, &display_info_list[4]);
	Display_Update(Buffer_Out_4_out1, &display_info_list[5]);
	Display_Update(Buffer_Out_5_out1, &display_info_list[6]);
}
