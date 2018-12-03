function [Imotor] = PoP_motor(Tmotor, Wmotor, Vmotor)
rps2rpm = 30/pi;
Jmotor = 0.0226;
Mmotor = 56.75;
Mgen = 32.70;
motor_description='PRIUS_JPN 50-kW permanent magnet motor/controller';
motor_eff = 0.95*[...
    0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000,0.2000;...
    0.4600,0.5400,0.5800,0.6000,0.6000,0.6000,0.6100,0.6350,0.6500,0.6600,0.6800,0.6950,0.7000,0.7100,0.7200,0.7200,0.7200,0.7100,0.6800,0.6000,0.4000,0.6000,0.6800,0.7100,0.7200,0.7200,0.7200,0.7100,0.7000,0.6950,0.6800,0.6600,0.6500,0.6350,0.6100,0.6000,0.6000,0.6000,0.5800,0.5400,0.4600;...
    0.6500,0.6600,0.7000,0.7200,0.7420,0.7500,0.7650,0.7800,0.7860,0.8000,0.8040,0.8200,0.8250,0.8270,0.8270,0.8250,0.8220,0.8150,0.7850,0.7200,0.5000,0.7200,0.7850,0.8150,0.8220,0.8250,0.8270,0.8270,0.8250,0.8200,0.8040,0.8000,0.7860,0.7800,0.7650,0.7500,0.7420,0.7200,0.7000,0.6600,0.6500;...
    0.7350,0.7400,0.7690,0.7800,0.7980,0.8090,0.8210,0.8270,0.8400,0.8430,0.8500,0.8550,0.8540,0.8630,0.8650,0.8640,0.8580,0.8480,0.8200,0.7680,0.5300,0.7680,0.8200,0.8480,0.8580,0.8640,0.8650,0.8630,0.8540,0.8550,0.8500,0.8430,0.8400,0.8270,0.8210,0.8090,0.7980,0.7800,0.7690,0.7400,0.7350;...
    0.7940,0.7940,0.8110,0.8200,0.8400,0.8500,0.8690,0.8670,0.8770,0.8780,0.8820,0.8830,0.8810,0.8870,0.8880,0.8860,0.8820,0.8720,0.8590,0.8000,0.6200,0.8000,0.8590,0.8720,0.8820,0.8860,0.8880,0.8870,0.8810,0.8830,0.8820,0.8780,0.8770,0.8670,0.8690,0.8500,0.8400,0.8200,0.8110,0.7940,0.7940;...
    0.8460,0.8460,0.8500,0.8560,0.8680,0.8760,0.8830,0.8880,0.8940,0.8970,0.9010,0.9040,0.9020,0.9070,0.9060,0.9040,0.8960,0.8830,0.8610,0.8100,0.6400,0.8100,0.8610,0.8830,0.8960,0.9040,0.9060,0.9070,0.9020,0.9040,0.9010,0.8970,0.8940,0.8880,0.8830,0.8760,0.8680,0.8560,0.8500,0.8460,0.8460;...
    0.8860,0.8860,0.8860,0.8860,0.8920,0.8970,0.9010,0.9070,0.9080,0.9140,0.9170,0.9200,0.9230,0.9210,0.9180,0.9150,0.9080,0.8970,0.8720,0.8220,0.7000,0.8220,0.8720,0.8970,0.9080,0.9150,0.9180,0.9210,0.9230,0.9200,0.9170,0.9140,0.9080,0.9070,0.9010,0.8970,0.8920,0.8860,0.8860,0.8860,0.8860;...
    0.9130,0.9130,0.9130,0.9130,0.9130,0.9130,0.9120,0.9170,0.9210,0.9270,0.9300,0.9310,0.9310,0.9300,0.9270,0.9240,0.9160,0.9050,0.8800,0.8400,0.7200,0.8400,0.8800,0.9050,0.9160,0.9240,0.9270,0.9300,0.9310,0.9310,0.9300,0.9270,0.9210,0.9170,0.9120,0.9130,0.9130,0.9130,0.9130,0.9130,0.9130;...
    0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9260,0.9320,0.9350,0.9400,0.9400,0.9400,0.9380,0.9360,0.9320,0.9250,0.9120,0.8860,0.8420,0.7500,0.8420,0.8860,0.9120,0.9250,0.9320,0.9360,0.9380,0.9400,0.9400,0.9400,0.9350,0.9320,0.9260,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220;...
    0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9410,0.9440,0.9460,0.9470,0.9480,0.9460,0.9430,0.9360,0.9210,0.9000,0.8590,0.6800,0.8590,0.9000,0.9210,0.9360,0.9430,0.9460,0.9480,0.9470,0.9460,0.9440,0.9410,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380;...
    0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9480,0.9510,0.9520,0.9510,0.9500,0.9470,0.9310,0.9090,0.8520,0.5800,0.8520,0.9090,0.9310,0.9470,0.9500,0.9510,0.9520,0.9510,0.9480,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460,0.9460;...
    0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9450,0.9510,0.9520,0.9480,0.9380,0.9100,0.8600,0.5700,0.8600,0.9100,0.9380,0.9480,0.9520,0.9510,0.9450,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400;...
    0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9450,0.9510,0.9530,0.9480,0.9400,0.9100,0.8500,0.5400,0.8500,0.9100,0.9400,0.9480,0.9530,0.9510,0.9450,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380,0.9380;...
    0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9470,0.9530,0.9500,0.9410,0.9200,0.8700,0.5800,0.8700,0.9200,0.9410,0.9500,0.9530,0.9470,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430,0.9430;...
    0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9500,0.9500,0.9430,0.9250,0.8820,0.6000,0.8820,0.9250,0.9430,0.9500,0.9500,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450;...
    0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9460,0.9400,0.9210,0.8820,0.5600,0.8820,0.9210,0.9400,0.9460,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450,0.9450;...
    0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9430,0.9380,0.9200,0.8800,0.5400,0.8800,0.9200,0.9380,0.9430,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400,0.9400;...
    0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9390,0.9340,0.9170,0.8800,0.5400,0.8800,0.9170,0.9340,0.9390,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370,0.9370;...
    0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9360,0.9320,0.9140,0.8800,0.5600,0.8800,0.9140,0.9320,0.9360,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350;...
    0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9330,0.9200,0.8820,0.6400,0.8820,0.9200,0.9330,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350,0.9350;...
    0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9330,0.9210,0.8830,0.7000,0.8830,0.9210,0.9330,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300,0.9300;...
    0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9290,0.9190,0.8800,0.7200,0.8800,0.9190,0.9290,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270,0.9270;...
    0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9160,0.8700,0.7000,0.8700,0.9160,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260,0.9260;...
    0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9100,0.8640,0.6800,0.8640,0.9100,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230,0.9230;...
    0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9000,0.8400,0.6400,0.8400,0.9000,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220,0.9220];
% % Motor Data Files
y1_tmax = [200,200,200,200,200,194,186,161,142,122,103,90,77.5,70,63.5,58,52,49,45,43,40,37.5,34.3,32.9,31.8];
y2_tmin = -y1_tmax;
% y1_tmax = [319.325555855000,317.805555855000,317.785555855000,317.765555855000,317.745555855000,317.725555855000,317.705555855000,317.685555854000,317.665555855000,317.645555854000,317.625555855000,317.605555855000,317.585555854000,314.146157170000,305.749612370000,295.235193292000,283.964708260000,272.628995200000,261.582240610000,251.000630455000,240.963190182000,231.495095569000,222.591726337000,214.232365136000,206.388124625000,199.026566191000,192.114381515000,185.618922363000,179.509038687000,173.755499909000,168.331165721000,163.211008333000,158.372049125000,153.793248276000,149.455371651000,145.340849212000,141.433634036000,137.719066741000,134.183748191000,130.815421679000,127.602865170000,124.535793041000,121.604767435000,118.801117907000,116.116869515000,113.544677745000,111.077770249000,108.709894454000,106.435270459000,104.248548750000,102.144772023000,100.119341032000,98.1679836001000,96.2867268220000,94.4718718960000,92.7199714130000,91.0278087573000,89.3923794892000,87.8108744493000,86.2806644007000,84.7992861332000,83.3644296708000,81.9739267800000,80.6257403664000,79.3179548964000,78.0487674655000,76.8164798467000,75.6194910662000,74.4562905692000,73.3254520138000,72.2256275114000,71.1555424473000,70.1139905396000,69.0998293681000,68.1119762381000,67.1494043547000];
% y2_tmin = [-319.325555855000,-320.845555855000,-320.865555855000,-320.885555855000,-320.905555854000,-320.925555855000,-320.945555855000,-320.965555855000,-320.985555854000,-321.005555854000,-321.025555855000,-321.045555854000,-321.048422682000,-316.979715389000,-308.954701496000,-299.070374032000,-288.442909904000,-277.682017996000,-267.121812408000,-256.941075746000,-247.229042852000,-238.022717431000,-229.328626777000,-221.135761453000,-213.423378294000,-206.165740909000,-199.335001370000,-192.902937660000,-186.841979947000,-181.125792224000,-175.729575666000,-170.630198621000,-165.806220007000,-161.237848575000,-156.906865345000,-152.796526733000,-148.891459160000,-145.177552143000,-141.641854047000,-138.272472948000,-135.058483682000,-131.989842014000,-129.057305689000,-126.252362412000,-123.567164260000,-120.994468111000,-118.527581986000,-116.160316127000,-113.886938974000,-111.702137410000,-109.600980719000,-107.578887900000,-105.631598493000,-103.755145587000,-101.945831961000,-100.200207918000,-98.5150516454000,-96.8873510225000,-95.3142872063000,-93.7932197690000,-92.3216730234000,-90.8973236253000,-89.5179892182000,-88.1816181514000,-86.8862798749000,-85.6301564285000,-84.4115342804000,-83.2287971767000,-82.0804194149000,-80.9649596144000,-79.8810550590000,-78.8274165126000,-77.8028233178000,-76.8061189405000,-75.8362069775000,-74.8920471877000];
% y3_loss = [1.3611	1.1517	0.9423	0.7329	0.5235	0.3141	0.2094	0.1047	0	0.05	0.1	0.15	0.25	0.36	0.47	0.57	0.65
% 2.04165	1.72755	1.41345	1.09935	0.78525	0.47115	0.3141	0.15705	0	0.15705	0.3141	0.47115	0.78525	1.09935	1.41345	1.72755	2.04165
% 20.4204	17.2788	14.1372	10.9956	7.854	4.7124	3.1416	1.5708	2.0944	1.5708	3.1416	4.7124	7.854	10.9956	14.1372	17.2788	20.4204
% 1041.44	881.217	720.996	560.775	400.553	240.332	160.221	80.1107	106.814	80.1107	160.221	240.332	400.553	560.775	720.996	881.217	1041.44
% 2062.46	1745.16	1427.85	1110.55	793.253	475.952	317.301	158.65	211.534	158.65	317.301	475.952	793.253	1110.55	1427.85	1745.16	2062.46
% 3083.47	2609.09	2134.71	1660.33	1185.95	711.571	474.38	237.19	316.254	237.19	474.38	711.571	1185.95	1660.33	2134.71	2609.09	3083.47
% 4104.49	3473.03	2841.57	2210.11	1578.65	947.19	631.46	315.73	420.973	315.73	631.46	947.19	1578.65	2210.11	2841.57	3473.03	4104.49
% 5125.51	4336.97	3548.43	2759.89	1971.35	1182.81	788.54	394.27	525.693	394.27	788.54	1182.81	1971.35	2759.89	3548.43	4336.97	5125.51
% 6146.53	5200.91	4255.29	3309.67	2364.05	1418.43	945.62	472.81	630.413	472.81	945.62	1418.43	2364.05	3309.67	4255.29	5200.91	6146.53
% 7167.54	6064.84	4962.15	3859.45	2756.75	1654.05	1102.7	551.349	735.133	551.349	1102.7	1654.05	2756.75	3859.45	4962.15	6064.84	7167.54
% 8188.56	6928.78	5669	4409.23	3149.45	1889.67	1259.78	629.889	839.852	629.889	1259.78	1889.67	3149.45	4409.23	5669	6928.78	8188.56
% 9209.58	7792.72	6375.86	4959	3542.15	2125.29	1416.86	708.429	944.572	708.429	1416.86	2125.29	3542.15	4959	6375.86	7792.72	9209.58
% 10230.6	8656.66	7082.72	5508.78	3934.84	2360.91	1573.94	786.969	1049.29	786.969	1573.94	2360.91	3934.84	5508.78	7082.72	8656.66	10230.6
% 11251.6	9520.6	7789.58	6058.56	4327.54	2596.53	1731.02	865.509	1154.01	865.509	1731.02	2596.53	4327.54	6058.56	7789.58	9520.6	11251.6
% 12272.6	10384.5	8496.44	6608.34	4720.24	2832.15	1888.1	944.049	1258.73	944.049	1888.1	2832.15	4720.24	6608.34	8496.44	10384.5	12272.6
% 13293.6	11248.5	9203.3	7158.12	5112.94	3067.77	2045.18	1022.59	1363.45	1022.59	2045.18	3067.77	5112.94	7158.12	9203.3	11248.5	13293.6
% 14334	12130	9924	7718	5514	3308	2206	1102	1470	1102	2206	3308	5514	7718	9924	12130	14334
% 16376	13858	11338	8818	6298	3780	2520	1260	1680	1260	2520	3780	6298	8818	11338	13858	16376
% 18420	15586	12752	9918	7084	4250	2834	1416	1888	1416	2834	4250	7084	9918	12752	15586	18420];
x1_w = 0:500:12000;
x2_w = x1_w;
x3_w = x1_w;
x3_t = -200:10:200;
% y3_loss = (1-motor_eff).*(x3_w'*x3_t)/rps2rpm;
% Internal Variables
Tmax = interp1(x1_w, y1_tmax, abs(Wmotor), 'linear', 'extrap');
Tmin = interp1(x2_w, y2_tmin, abs(Wmotor), 'linear', 'extrap');

Eff = zeros(1,length(Wmotor))*NaN;
% Loss(Wmotor<=7500) = interpextrap2(x3_t,x3_w,y3_loss, Tmotor, Wmotor);
% Imotor = zeros(1,length(Wmotor))*NaN;
feas = (Tmotor<=Tmax)&(Tmotor>=Tmin)&(abs(Wmotor)<=12000);
% Tmotor = Tmotor.*(Tmotor<Tmax&Tmotor>Tmin) + Tmax.*(Tmotor>=Tmax) + Tmin.*(Tmotor<=Tmin);

Eff(feas) = interp2(x3_t,x3_w,motor_eff, Tmotor(feas), abs(Wmotor(feas)), 'linear', 0.1);

Loss_m = (1./Eff - 1).*Tmotor.*Wmotor/rps2rpm;
Loss_g = (Eff-1).*Tmotor.*Wmotor/rps2rpm;
Pmot = Tmotor.*Wmotor;

Loss = Loss_m.*(Pmot>0)+Loss_g.*(Pmot<=0);

% Imotor(feas) = -(Tmotor(feas).*Wmotor(feas)/rps2rpm+Loss(feas))/Vmotor;
Imotor = -(Tmotor.*Wmotor/rps2rpm+Loss)/Vmotor;
% % % % if (Tmotor<=Tmax && Tmotor >=Tmin) && (Wmotor <=9000)
% % % %     % Output Calculations
% % % %     Loss = interpextrap2(x3_t,x3_w,y3_loss, Tmotor, Wmotor);   
% % % %     Imotor = -(Tmotor*Wmotor/rps2rpm+Loss)/Vmotor;
% % % % else
% % % %     Imotor = NaN;
% % % % end
end