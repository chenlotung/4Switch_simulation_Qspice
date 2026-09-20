// Automatically generated C++ file for QSPICE
//
// Build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD -o digital_control.cpp kernel32.lib
//

// ============================================================
// Controller selection
// ============================================================

#define CONTROL_2P2Z  1
#define CONTROL_PID    2

// Select controller here:
#define CONTROL_MODE  CONTROL_2P2Z//CONTROL_PID//

// ============================================================

union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};


// Required by QSPICE DLL
int __stdcall DllMain(
   void *module,
   unsigned int reason,
   void *reserved)
{
   return 1;
}


// Avoid possible conflicts with headers
#undef Vin
#undef Vout
#undef PWMA1
#undef PWMA2
#undef PWMB1
#undef PWMB2
#undef Iin
#undef I_inductor


extern "C" __declspec(dllexport)
void digital_control(
   void **opaque,
   double t,
   union uData *data)
{
   // =========================================================
   // QSPICE I/O
   // =========================================================

   double  Vin        = data[0].d; // input voltage
   double  Vout       = data[1].d; // output voltage
   double  Iin        = data[2].d; // input current
   double  I_inductor = data[3].d; // inductor current

   double &PWMA1      = data[4].d;
   double &PWMA2      = data[5].d;
   double &PWMB1      = data[6].d;
   double &PWMB2      = data[7].d;


   // Suppress unused-input warnings for now.
   // Later Iin / I_inductor can be used for current limit.
   (void)Iin;
   (void)I_inductor;


   // =========================================================
   // Power stage / PWM configuration
   // =========================================================

   const double FSW       = 250000.0;
   const double TSW       = 1.0 / FSW;

   const double DEADTIME  = 100e-9;

   const double GATE_HIGH = 10.0;


   // =========================================================
   // Voltage control configuration
   // =========================================================

   const double VREF_FINAL = 48.0;

   // Input reaches 60 V at 5 ms in your present schematic.
   const double ENABLE_TIME = 5e-3;

   // 5 ms soft-start:
   // 5 ms -> 10 ms : 0 V -> 48 V
   const double SOFTSTART_TIME = 5e-3;


   // Duty limits
   const double DUTY_MIN = 0.05;
   const double DUTY_MAX = 0.95;


   // =========================================================
   // Persistent controller states
   // =========================================================

   static long long last_cycle = -1;

   static double duty = 0.0;


#if CONTROL_MODE == CONTROL_2P2Z

   // =========================================================
   // 2P2Z STATES
   //
   // Controller equation:
   //
   // u[n] =
   //      A1*u[n-1]
   //    + A2*u[n-2]
   //    + B0*e[n]
   //    + B1*e[n-1]
   //    + B2*e[n-2]
   //
   // duty = duty_feedforward + u
   //
   // Error is NORMALIZED:
   //
   // e = (Vref - Vout) / 48V
   // =========================================================

   static double error_z1 = 0.0;
   static double error_z2 = 0.0;

   static double ctrl_z1 = 0.0;
   static double ctrl_z2 = 0.0;


   // ---------------------------------------------------------
   // Initial 2P2Z coefficients
   //
   // These are starting coefficients for simulation tuning.
   // Because error is normalized, controller output is in duty.
   // ---------------------------------------------------------

   const double A1 =  1.59830271;
   const double A2 = -0.59830271;

   const double B0 =  0.00450;
   const double B1 = -0.00882;
   const double B2 =  0.00432;


#elif CONTROL_MODE == CONTROL_PID

   // =========================================================
   // PID STATES
   //
   // Normalized error:
   //
   // error = (Vref - Vout) / 48
   //
   // duty correction:
   //
   // u = Kp*e + integral + Kd*de/dt
   // =========================================================

   static double error_z1 = 0.0;
   static double integral = 0.0;


   // Initial conservative PID values
   const double KP = 0.18;
   const double KI = 500.0;
   const double KD = 2.0e-6;


#else

#error Invalid CONTROL_MODE

#endif


   // =========================================================
   // Converter OFF before Vin startup is complete
   // =========================================================

   if ((t < ENABLE_TIME) || (Vin < 5.0))
   {
      PWMA1 = 0.0;
      PWMA2 = 0.0;
      PWMB1 = 0.0;
      PWMB2 = 0.0;

      duty = 0.0;

      last_cycle = (long long)(t / TSW);


#if CONTROL_MODE == CONTROL_2P2Z

      error_z1 = 0.0;
      error_z2 = 0.0;

      ctrl_z1 = 0.0;
      ctrl_z2 = 0.0;


#elif CONTROL_MODE == CONTROL_PID

      error_z1 = 0.0;
      integral = 0.0;

#endif

      return;
   }


   // =========================================================
   // Determine present PWM cycle
   // =========================================================

   long long cycle =
      (long long)(t / TSW);


   // =========================================================
   // Digital controller
   //
   // IMPORTANT:
   //
   // QSPICE may call this DLL many times inside one switching
   // period.
   //
   // Controller MUST only update once per PWM period.
   // =========================================================

   if (cycle != last_cycle)
   {
      last_cycle = cycle;


      // ======================================================
      // Soft-start reference
      // ======================================================

      double softstart =
         (t - ENABLE_TIME) /
         SOFTSTART_TIME;

      if (softstart > 1.0)
         softstart = 1.0;

      if (softstart < 0.0)
         softstart = 0.0;


      double Vref =
         VREF_FINAL * softstart;


      // ======================================================
      // Normalized voltage error
      // ======================================================

      double error = 0.0;

      if (VREF_FINAL > 0.0)
      {
         error =
            (Vref - Vout) /
            VREF_FINAL;
      }


      // ======================================================
      // Input voltage feed-forward
      //
      // Ideal Buck:
      //
      // D = Vout / Vin
      // ======================================================

      double duty_ff = 0.0;

      if (Vin > 1.0)
      {
         duty_ff = Vref / Vin;
      }


      // Avoid impossible feed-forward request
      if (duty_ff > DUTY_MAX)
         duty_ff = DUTY_MAX;

      if (duty_ff < DUTY_MIN)
         duty_ff = DUTY_MIN;



#if CONTROL_MODE == CONTROL_2P2Z

      // ======================================================
      // 2P2Z controller
      // ======================================================

      double controller =
           A1 * ctrl_z1
         + A2 * ctrl_z2
         + B0 * error
         + B1 * error_z1
         + B2 * error_z2;


      double duty_raw =
         duty_ff + controller;


      // ======================================================
      // Duty saturation
      // ======================================================

      if (duty_raw > DUTY_MAX)
      {
         duty = DUTY_MAX;
      }
      else if (duty_raw < DUTY_MIN)
      {
         duty = DUTY_MIN;
      }
      else
      {
         duty = duty_raw;
      }


      // ======================================================
      // Anti-windup
      //
      // Force controller state to correspond to actual
      // saturated duty.
      // ======================================================

      controller =
         duty - duty_ff;


      // Update histories
      error_z2 = error_z1;
      error_z1 = error;

      ctrl_z2 = ctrl_z1;
      ctrl_z1 = controller;



#elif CONTROL_MODE == CONTROL_PID

      // ======================================================
      // PID controller
      // ======================================================

      double proportional =
         KP * error;


      // Candidate integrator state
      double integral_candidate =
         integral +
         KI * error * TSW;


      double derivative =
         KD *
         (error - error_z1) /
         TSW;


      double controller =
           proportional
         + integral_candidate
         + derivative;


      double duty_raw =
         duty_ff + controller;


      // ======================================================
      // Duty saturation
      // ======================================================

      double duty_limited =
         duty_raw;


      if (duty_limited > DUTY_MAX)
         duty_limited = DUTY_MAX;

      if (duty_limited < DUTY_MIN)
         duty_limited = DUTY_MIN;


      // ======================================================
      // Conditional integration anti-windup
      //
      // If saturated HIGH and error wants duty even higher:
      // do not integrate.
      //
      // If saturated LOW and error wants duty even lower:
      // do not integrate.
      // ======================================================

      bool allow_integrator = true;


      if ((duty_raw > DUTY_MAX) &&
          (error > 0.0))
      {
         allow_integrator = false;
      }


      if ((duty_raw < DUTY_MIN) &&
          (error < 0.0))
      {
         allow_integrator = false;
      }


      if (allow_integrator)
      {
         integral =
            integral_candidate;
      }


      // Recalculate controller using accepted integrator
      controller =
           proportional
         + integral
         + derivative;


      duty =
         duty_ff + controller;


      // Final clamp
      if (duty > DUTY_MAX)
         duty = DUTY_MAX;

      if (duty < DUTY_MIN)
         duty = DUTY_MIN;


      // Save error history
      error_z1 = error;


#endif


      // ======================================================
      // During very early soft-start, allow true 0 duty
      // ======================================================

      if (Vref < 0.5)
      {
         duty = 0.0;
      }
   }


   // =========================================================
   // PWM generation
   // =========================================================

   double phase =
      t - ((double)cycle * TSW);


   double ton =
      duty * TSW;


   // Default OFF
   PWMA1 = 0.0;
   PWMA2 = 0.0;


   // =========================================================
   // BUCK MODE
   //
   // Left bridge:
   //
   // M1 = PWM
   // M2 = complementary PWM
   //
   // Right bridge:
   //
   // M3 = always ON
   // M4 = always OFF
   // =========================================================

   PWMB1 = GATE_HIGH;
   PWMB2 = 0.0;


   // =========================================================
   // M1 high-side PWM
   // =========================================================

   if (ton > DEADTIME)
   {
      if ((phase >= (DEADTIME * 0.5)) &&
          (phase <  (ton - DEADTIME * 0.5)))
      {
         PWMA1 =
            GATE_HIGH;
      }
   }


   // =========================================================
   // M2 complementary PWM
   // =========================================================

   if ((TSW - ton) > DEADTIME)
   {
      if ((phase >= (ton + DEADTIME * 0.5)) &&
          (phase <  (TSW - DEADTIME * 0.5)))
      {
         PWMA2 =
            GATE_HIGH;
      }
   }
}
