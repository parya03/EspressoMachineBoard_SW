/**
 * File for PID control of temp, brew control, etc
 * 
 * 
 */

#include "Control.h"

adc_oneshot_unit_handle_t adc1_handle = NULL;
adc_cali_handle_t adc1_cali_handle = NULL;
// int u = 0;

// Boiler IO
gpio_config_t boiler_io_conf = {
    .pin_bit_mask = ((1ULL << IO_BOILER)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = (gpio_pullup_t)GPIO_PULLUP_DISABLE,
    .pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

float setpoint = 0.0f; // Degrees C
float curr_temp = 100.0f; // Start high by default for safety

// Read raw ADC average value
static int get_therm_adc_reading() {
    float therm_raw_avg_adc = 0;

    for(int i = 0; i < 100; i++) {
        int therm_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &therm_raw));

        therm_raw_avg_adc += therm_raw;
        // ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, therm_raw, &therm_mV));

        // therm_mV += 30; // Experimentally determined to generally be measured as ~30mV lower than actual (multimeter reference)
    }

    therm_raw_avg_adc /= 100.0f;

    return therm_raw_avg_adc;
}

static float adc_2_resistance(int adc) {
    return 794.88817 + (-214.44469 * log10f(adc)) + (-219.13082 * expf(-0.000495547 * adc));
}

static float adc_2_temp(int adc) {
    // Calculate temp
    // Find resistance based off of reference voltage output (measured as 2.601v)
    // float resistance = ((2.601f * 10000.0f) / (float)(therm_mV/1000.0f)) - 10000.0f;
    // Exponential regression (experimental) based off of raw ADC values seems to work well
    // float resistance_kohms = (184.9902f*exp(-0.00371704f*therm_raw_avg)+15.55841f);
    float resistance_kohms = 794.88817 + (-214.44469 * log10f(adc)) + (-219.13082 * expf(-0.000495547 * adc));

    // Playing with data for a 100k NTC thermistor in Desmos gives this weird regression as best (for above 40C):
    // 44.81207 + -0.000144689x + 110.23167*e^(-0.000103527 * x)

    // Experimental regression: Numbers work until ~65C then we do linefar from there
    // curr_temp = 154.1806f + (-69.60769f * log10(resistance_kohms)) + (0.0740364f * resistance_kohms);
    // if(resistance_kohms > 19)
    //     curr_temp = 161.14291 + (-75.90308 * log10f(resistance_kohms)) + (0.151561 * resistance_kohms);
    // else
    //     curr_temp = (-8.41398 * resistance_kohms) + 229.19328;

    // Steinhard-Hart might not work because of FP shenanegans
    // curr_temp = 1.0f / (0.002036510094 + (0.0003240228966 * logf(resistance_kohms)) + (-0.000001519537509 * pow(logf(resistance_kohms), 3)));
    // B Model - Convert to degrees K for this
    return (1.0f / ((1.0f/298.15f) + ((1.0f/3710.83f)*logf(resistance_kohms/84.39f)))) - 273.15;
}

void control_init() {
    esp_err_t ret;
    
    // PWM config for boiler
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 2,  // Set output frequency at 2Hz so we only control duty cycle
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = IO_BOILER,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // ADC1 Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Config
    adc_oneshot_chan_cfg_t adc1_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &adc1_config));

    // ADC1 Calibration
    ESP_LOGI("", "Calibrating ADC1 Channel 1 (thermistor)...");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    // if (ret == ESP_OK) {
    //     calibrated = true;
    // }
}

// Return logistic function 0 <= y <= 1 corresponding to magnitude of energy change reported by y(t)
// Multiply this by energy put in (proportional to u(t)) to get the energy reading (temperature) at future sample n
static float y_logistic(float n) {
    return (float)((exp(0.125f * (n - MCP_N))) / (1 + exp(0.125f * (n - MCP_N))));
}

// n is in seconds, d_Q is in Joules. Return value of expected temperature input at n seconds from now.
static float y_hyperbola(float n, float Q, float d_Q) {
    float hyperbola = (1.0f / (-0.1f * (n - 4.0f))) + (d_Q/MODEL_J_PER_C);

    if(hyperbola < 0.0f) {
        return (Q/MODEL_J_PER_C);
    }
    else {
        return (Q/MODEL_J_PER_C) + hyperbola;
    }
}

// n is in seconds, Q and dQ_cmd are arrays in Joules
// Return value of expected temperature input at n seconds from now based on previous state and control history
// buffer_element_num is how many elements are in the buffers given, since they won't necessarily be full e.g. on startup
static float y_exp_use_hist(float n, float Q[MCP_N], int Q_index, float dQ_cmd[MCP_N], int dQ_index, int buffer_element_num) {
    float state_Q = 0.0f;
    
    for(int i = 1; i <= buffer_element_num; i++) {
        float hist_Q = Q[(Q_index - i) % MCP_N];
        float hist_dQ_cmd = dQ_cmd[(dQ_index - i) % MCP_N];

        float hist_T = hist_Q / MODEL_J_PER_C;
        float hist_dT = hist_dQ_cmd / MODEL_J_PER_C;
        
        float state_diff = hist_T + (hist_dT - (exp(-0.125 * (i + (n * SAMPLES_PER_SEC) - 2)) * hist_dT));
        // printf("Hist T = %f, Hist dT = %f, hist_Q = %f, hist_dQ_cmd=%f, state_diff=%f\n", hist_T, hist_dT, hist_Q, hist_dQ_cmd, state_diff);

        state_Q += (state_diff >= 0) ? state_diff : 0;
    }

    return (state_Q / (float)buffer_element_num); // Average of all predicted values
}

static Matrix gen_A_matrix_jacobian(Matrix x, int u, bool pump_on) {
    return Matrix(2, 2, (float[100]){1, x[1][0] * (PID_TIME_MS/1000.0f), 0, 0});
}

static Matrix x_state_transition(Matrix x, int u, bool pump_on) {
    return Matrix(2, 1, (float[100]){x[0][0] + ((PID_TIME_MS / 1000.0f) * x[1][0]), ((-625.304f * (int)pump_on) + ((PID_TIME_MS / SSR_TIME_MS) * (ENERGY_PER_HALF_PHASE * u)))});
}                                                                                                                                                

void temp_control_task(void *pvParameters) {
    float therm_raw_avg = 0.0f;
    float temp_prev = 0.0f;

    // PID stuff
    float int_e = 0; // Integral
    float prev_e = 0; // Used for derivative
    float d_e = 0; // Derivative

    // MPC state variables
    // Current energy in the thermoblock, relative to 0 C
    // Assumes steady state initially. If it's not steady state, then control system should (hopefully) compensate later on by updating state.
    float curr_energy = adc_2_temp(get_therm_adc_reading()) * MODEL_J_PER_C;

    float Y_pred[MCP_N] = { 0 };
    float dY_pred[MCP_N] = { 0 };
    int dY_pred_index = 1; // It's a ring buffer

    // Kalman state variables
    // Row-major order e.g. X[row][col]
    Matrix X(2, 1);
    Matrix P(2, 2);

    // Kalman constants
    // Matrix A(2, 2, (float[100]){1, PID_TIME_MS/1000.0f, 0.0f, 1.0f});
    Matrix A = gen_A_matrix_jacobian(X, 0, false); // Initial A matrix
    A.print();
    A.transpose().print();
    // Matrix A_transpose[2][2] = {{1, 1}, {1, PID_TIME_MS/1000.0f}};
    Matrix Q(2, 2, (float[100]){5.0f, 0.0f, 0.0f, 0.5f});
    
    // Previous predictions and control inputs
    // Ring buffers
    float commanded_dQ_prev[MCP_N] = { 0 };
    int commanded_dQ_prev_index = 0;
    float Q_prev[MCP_N] = { 0 };
    int Q_prev_index = 0;

    // Matrix test1(2, 3, (float[100]){1, 2, 3, 4, 5, 6});
    // Matrix test2(3, 2, (float[100]){7, 8, 9, 10, 11, 12});

    // test1.print();
    // test2.print();
    // (test1 * test2).print();

    Matrix test3(2, 1, (float[100]){1, 2});
    Matrix test4(2, 1, (float[100]){1, 2});

    test3.print();
    test4.print();
    test4 = (test3 + test4);
    test4.print();

    // Get first 2 measurements
    X[0][0] = adc_2_temp(get_therm_adc_reading()) * MODEL_J_PER_C;
    P[0][0] = 0.02; // 0.279f;
    vTaskDelay(PID_TIME_MS / portTICK_PERIOD_MS);
    float curr_X0 = adc_2_temp(get_therm_adc_reading()) * MODEL_J_PER_C;
    X[1][0] = curr_X0 - X[0][0];
    X[0][0] = curr_X0;
    P[1][1] = 0.05f; // 100; // High variance bc we aren't sure about change yet
    ESP_LOGI("Control - Kalman", "Initial states generated, X = [%f, %f]T; P = [%f, %f; %f, %f]", X[0][0], X[1][0], P[0][0], P[0][1], P[1][0], P[1][1]);
    vTaskDelay(PID_TIME_MS / portTICK_PERIOD_MS);

    int u = 0;

    // Initial Prediction step
    Matrix X_p = x_state_transition(X, 0, false);
    Matrix P_p = ((A * P) * A.transpose()) + Q;
    
    Q_prev[(Q_prev_index++) % MCP_N] = X[0][0]; 
    commanded_dQ_prev[(commanded_dQ_prev_index++) % MCP_N] = 0; // No control input given yet

    // Control input
    // extern uint8_t pump_state;
    // X_p[0][0] += (PID_TIME_MS / SSR_TIME_MS) * (ENERGY_PER_HALF_PHASE * u);
    // X_p[1][0] += (-625.304f * pump_state) + ((1000.0f / SSR_TIME_MS) * ENERGY_PER_HALF_PHASE * u); // Energy loss from pump putting water through TODO div 2 to account for timestep?    
    
    int iterations = 0; // How many iterations of the loop we have done, to account for some buffers not being full on startup
    // Measure thermistor and output on serial
    while(1) {
        iterations++;

        therm_raw_avg = get_therm_adc_reading();
        curr_temp = adc_2_temp(therm_raw_avg);

        // ESP_LOGI("ADC", "Thermistor measured at raw=%f = %d mV, R=%f kOhms, Temp=%f C, dTemp = %f", therm_raw_avg, therm_mV, resistance_kohms, temp, (temp - temp_prev));

        // TODO measure energy better
        // float setpoint_energy_error = (setpoint * MODEL_J_PER_C) - curr_energy; // How much more energy we need to dump into thermoblock to meet setpoint
        float setpoint_energy_error = (setpoint * MODEL_J_PER_C) - X[0][0]; // How much more energy we need to dump into thermoblock to meet setpoint
        float setpoint_energy = (setpoint * MODEL_J_PER_C);

        static int count = 0;
        // ------------------------------------------------------------------
        // Kalman filter

        // Estimate step
        Matrix S(1, 1);
        // Matrix H(1, 2, (float[100]){y_logistic(PID_TIME_MS/1000.0f) / MODEL_J_PER_C, ((PID_TIME_MS/1000.0f) * y_logistic(PID_TIME_MS/1000.0f)) / MODEL_J_PER_C});
        // Matrix H(1, 2, (float[100]){1.0f / MODEL_J_PER_C, (1.0f / MODEL_J_PER_C) - (0.004107)}); // dT/dQ = 1/J_per_C, dT/d(dQ) = (1 / J_PER_C) - exp(0.25) / J_PER_C
        Matrix H(1, 2, (float[100]){1.0f / MODEL_J_PER_C, 0.0008539846219f}); // dT/dQ = 1/J_per_C, dT/d(dQ) = (1 / J_PER_C) - exp(0.25) / J_PER_C
        S = ((H * P_p) * H.transpose()) + Matrix(1, 1, 0.02); // 0.279f
        Matrix K(2, 1);
        Matrix S_inv(1, 1);
        S_inv[0][0] = 1.0f / S[0][0]; // Sketchy non-generic trick but we know S is 1x1
        K = ((P_p * H.transpose()) * S_inv);

        Matrix Z(1, 1, curr_temp);
        // Matrix hx(1, 1, ((X[0][0] / MODEL_J_PER_C) + ((X_p[1][0] * y_logistic(PID_TIME_MS/1000.0f))/MODEL_J_PER_C))); // z - h(x_p)
        // Matrix hx(1, 1, y_hyperbola(PID_TIME_MS / 1000.0f, X_p[0][0], X_p[1][0])); // z - h(x_p)
        int buf_elements = (iterations >= 50) ? 50 : iterations;
        // Matrix hx(1, 1, y_exp_use_hist(PID_TIME_MS / 1000.0f, Q_prev, Q_prev_index, commanded_dQ_prev, commanded_dQ_prev_index, buf_elements)); // z - h(x_p)
        Matrix hx(1, 1, y_exp_use_hist(0, Q_prev, Q_prev_index, commanded_dQ_prev, commanded_dQ_prev_index, buf_elements)); // z - h(x_p)
        X = X_p + (K * (Z - hx));
        P = P_p - ((K * H) * P_p);

        (K * H).print();
        X_p.print();
        X.print();
        K.print();
        H.print();
        H.transpose().print();
        hx.print();
        P_p.print();
        S.print();
        (K * (Z - hx)).print();
        ((K * H) * P_p).print();
        ((A * P)).print();
        
        // Store prediction
        Q_prev[(Q_prev_index++) % MCP_N] = X[0][0]; 

        Matrix X_deviation(2, 1, (float[100]){X[0][0] - X_p[0][0], X[1][0] - X_p[1][0]});
        X_deviation.print();
        ESP_LOGI("Control - Kalman", "hx[0] = %f, States generated, X = [%f, %f]T; P = [%f, %f; %f, %f]", hx[0][0], X[0][0], X[1][0], P[0][0], P[0][1], P[1][0], P[1][1]);
        // ------------------------------------------------------------------

        // MPC - Model Predictive Control
        // We use a model of the system to predict what the state will be later on and use that to control
        // Temperature measurement follows logistic function:
        // x[n] = (energy / 550 J/C)*(e^(0.125(n-50)))/(1+e^(0.125(n-50)))
        // Where N is in HALF SECONDS
        float U[MCP_N] = { 0.0f }; // Array of predicted future control inputs
        float MCP_X[MCP_N] = { X[0][0] }; // Predicted future energy states where X[0] is current state
        
        // Initial error calculation
        float MSE = 0.0f;
        for(auto x : MCP_X) {
            MSE += powf(x - setpoint, 2);
        }
        MSE /= (2.0f * (float)MCP_N); // 1/2N to make gradient easier to compute

        // Based off model, calculate control sequence to minimize MSE
        for(int n = 0; n < MCP_N - 1; n++) { // End at MCP_N - 1 so that X doesn't overflow. We still predict quite far ahead.
            // Calculate what value of u can minimize gradient of error
            // Gradient: (x(n) - r) / N for all n in X

            // Calculate optimal control to minimize error
            U[n] = (setpoint_energy - MCP_X[n]) / ENERGY_PER_HALF_PHASE;
            // Bounds check
            if(U[n] < 0) {
                U[n] = 0;
            }
            if(U[n] > 60) {
                U[n] = 60;
            }
            // Predict next state
            MCP_X[n+1] = MCP_X[n] + (ENERGY_PER_HALF_PHASE * U[n]) - X_deviation[1][0]; // Account for current heat loss
            // Update MSE
            MSE += (MCP_X[n+1] - setpoint_energy) / (float)MCP_N;
        }

        // Only output first predicted control output
        u = U[0];

        u = (u < 0) ? 0 : (u > 60) ? 60 : u; // Bounds checking

        // Update energy put in
        curr_energy += (u * ENERGY_PER_HALF_PHASE); // Each half phase of control adds Joules to the system.

        // Store control input
        commanded_dQ_prev[(commanded_dQ_prev_index++) % MCP_N] = X[1][0] + (u * ENERGY_PER_HALF_PHASE);
        
        // Update predicted y(t) and dy/dt based on the calculated control input
        // Zero out last index in ring buffer since that's the new "frontier" of our current ring buffer
        // Y_pred[(dY_pred_index - 1) % MCP_N] = 0.0f;
        // dY_pred[(dY_pred_index - 1) % MCP_N] = 0.0f;
        // // TODO this part is broken
        // for(int i = 0; i < MCP_N; i++) {
        //     // Derivative of C * logistic = C * dLogistic = C * logistic * (1 - logistic)
        //     // They also add up for each summation
        //     Y_pred[(dY_pred_index + i + 1) % MCP_N] += ((u * ENERGY_PER_HALF_PHASE) * (y_logistic(i))) / MODEL_J_PER_C;
        //     dY_pred[(dY_pred_index + i + 1) % MCP_N] = Y_pred[(dY_pred_index + i + 1) % MCP_N] - Y_pred[(dY_pred_index + i) % MCP_N];
        // }

        // // u += ((brew_state && pump_state) * 60); // Take into account brew/pump state to compensate for cold water being pumped into boiler
        // u = (u < 0) ? 0 : (u > 60) ? 60 : u; // Bounds checking

        // ESP_LOGI("Temp PID", "y(t)=%f, e(t)=%f, u(t)=%f, P=%f, I=%f, D=%f, int_e=%f, d_e=%f", temp, e, u, p, i, d, int_e, d_e);

        // ESP_LOGI("Temp PID", "t=%lu, r(t)=%f, y(t)=%f, e(t)=%f, u(t)=%d half-phases, resistance=%f kohms, raw ADC=%f, P=%f, I=%f, D=%f, Kc=%f, int_e=%f, d_e=%f", (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()), setpoint, curr_temp, e, u, resistance_kohms, therm_raw_avg, p, i, d, kc, int_e, d_e);
        ESP_LOGI("Temp MPC", "t=%lu, r(t)=%f, y(t)=%fC=%fJ, x(t)=%f Joules, Setpoint energy=%f, MSE=%f, u(t)=%d half-phases, dy(t)=%f, resistance=%f kohms, raw ADC=%f, d_e=%f", (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()), setpoint, curr_temp, curr_temp*MODEL_J_PER_C, curr_energy, setpoint_energy, MSE, u, (curr_temp - temp_prev), adc_2_resistance(therm_raw_avg), therm_raw_avg, d_e);
        // ESP_LOGI("Temp MPC", "Next 10 u: %f %f %f %f %f %f %f %f %f %f", U[0], U[1], U[2], U[3], U[4], U[5], U[6], U[7], U[8], U[9]);
        // ESP_LOGI("Temp MPC", "Next 10 MCP_X: %f %f %f %f %f %f %f %f %f %f", MCP_X[0], MCP_X[1], MCP_X[2], MCP_X[3], MCP_X[4], MCP_X[5], MCP_X[6], MCP_X[7], MCP_X[8], MCP_X[9]);
        // ESP_LOGI("Temp MPC", "Next 10 dY (predicted): %f %f %f %f %f %f %f %f %f %f", dY_pred[(dY_pred_index + 0) % MCP_N], dY_pred[(dY_pred_index + 1) % MCP_N], dY_pred[(dY_pred_index + 2) % MCP_N], dY_pred[(dY_pred_index + 3) % MCP_N], dY_pred[(dY_pred_index + 4) % MCP_N], dY_pred[(dY_pred_index + 5) % MCP_N], dY_pred[(dY_pred_index + 6) % MCP_N], dY_pred[(dY_pred_index + 7) % MCP_N], dY_pred[(dY_pred_index + 8) % MCP_N], dY_pred[(dY_pred_index + 9) % MCP_N]);
    
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 273*u);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        // ------------------------------------------------------------------
        // Kalman Filter - prediction step
        // Prediction step
        // X_p = A * X;
        extern uint8_t pump_state;
        A = gen_A_matrix_jacobian(X, u, pump_state);
        // X_p = A * ((Matrix(2, 1, (float[100]){X[0][0], (-625.304f * pump_state) + ((1000.0f / SSR_TIME_MS) * ENERGY_PER_HALF_PHASE * u)})));
        // X_p = A * ((Matrix(2, 1, (float[100]){X[0][0], X[1][0]})));
        X_p = x_state_transition(X, u, pump_state);

        // Control input
        // X_p[0][0] += (-625.304f * pump_state) + ((PID_TIME_MS / SSR_TIME_MS) * (ENERGY_PER_HALF_PHASE * u));
        // X_p[1][0] += (-625.304f * pump_state) + ((1000.0f / SSR_TIME_MS) * ENERGY_PER_HALF_PHASE * u); // Energy loss from pump putting water through TODO div 2 to account for timestep?

        X_p.print();

        // Matrix P_p(2, 2);
        P_p = ((A * P) * A.transpose()) + Q;
        // ------------------------------------------------------------------
        
        // dY_pred_index += 1;
        temp_prev = curr_temp;

        vTaskDelay(PID_TIME_MS / portTICK_PERIOD_MS);
    }
}
