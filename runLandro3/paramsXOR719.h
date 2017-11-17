String ID = "A2E"; 
#define NUM_INPUT 6
#define NUM_HIDDEN 9 
#define NUM_OUTPUT 2 
 
float input[NUM_INPUT]; 
float hidden[NUM_HIDDEN]; 
float old_hidden[NUM_HIDDEN]; 
float output[NUM_OUTPUT]; 
float old_output[NUM_OUTPUT]; 
 
const int RMILength = 1; 
const int LMILength = 1; 
int RMI[RMILength] = {1}; 
int LMI[LMILength] = {0}; 
 
int sensor_to_input[NUM_INPUT] = {2,3,0,1,14,15}; 
float input_to_hidden[NUM_INPUT][NUM_HIDDEN] = {{3,-3,0,0,0,0,0,0,0},{-3,3,0,0,0,0,0,0,0},{0,0,0,3,-3,0,0,0,0},
												                        {0,0,0,-3,3,0,0,0,0},{0,0,0,0,0,0,3,-3,0},{0,0,0,0,0,0,-3,3,0}}; 

float hidden_to_hidden[NUM_HIDDEN][NUM_HIDDEN] = {{0,0,3,0,0,0,0,0,0},{0,0,3,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0},
                        												  {0,0,0,0,0,3,0,0,0},{0,0,0,0,0,3,0,0,0},{0,0,0,0,0,0,0,0,0},
                        											    {0,0,0,0,0,0,0,0,3},{0,0,0,0,0,0,0,0,3},{0,0,0,0,0,0,0,0,0}};

float hidden_to_output[NUM_HIDDEN][NUM_OUTPUT] = {{0,0},{0,0},{-0.75,3},
                        												  {0,0},{0,0},{-0.5,-0.5},
                        												  {0,0},{0,0},{3,-0.75}}; 

float input_to_output[NUM_INPUT][NUM_OUTPUT] = {{0,0},{0,0},{0,0},
												                        {0,0},{0,0},{0,0}}; 

float output_to_hidden[NUM_OUTPUT][NUM_HIDDEN] = {{0,0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0,0}}; 
 
 
 
