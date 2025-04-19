data = readtable("D:\Major Project\Quadcopter\Pitch\combined_exp1_1.csv");
inputs_id = [data.m1, data.m2, data.m3, data.m4];
outputs_id = [data.pitch, data.velocity];
Ts = 0.01;
id = iddata(outputs_id, inputs_id, Ts);

data = readtable("D:\Major Project\Quadcopter\Pitch\combined_exp2_1.csv");
inputs_val = [data.m1, data.m2, data.m3, data.m4];
outputs_val = [data.pitch, data.velocity];
val = iddata(outputs_val, inputs_val, Ts);