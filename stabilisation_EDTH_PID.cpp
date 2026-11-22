// =====================================================================
//  Stabilisation 2 axes - Proteus Group - EDTH Paris 2026 - Défi 02
//  IMU sur la base (= attitude du drone, rôle du Cube Orange)
//  1) ItTest_EDTH_calculate : filtre complémentaire (fonction de Paul, corrigée)
//  2) ItTest_EDTH_pid       : PID qui fait suivre la consigne aux servos (-angle du drone)
//     sans dépassement ni vibration ; le PID de position interne du servo ES09MD fait le reste.
// =====================================================================
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ---------------------- RÉGLAGES ------------------------------------
const int    PIN_SERVO_X    = 9;        // servo 1, axe X (roulis)   [à adapter]
const int    PIN_SERVO_Y    = 10;       // servo 2, axe Y (tangage)  [à adapter]
const double US_PAR_DEGRE_X = 10.0;     // 500 / angle mesuré entre 1500 et 2000 µs [à mesurer]
const double US_PAR_DEGRE_Y = 10.0;
const int    NEUTRE_X_US    = 1500;     // plateau à plat
const int    NEUTRE_Y_US    = 1500;
const int    SENS_X         = -1;       // -1 ou +1 si le servo corrige dans le mauvais sens
const int    SENS_Y         = -1;
const double GAIN           = 1.0;      // compensation : baisser (0.8, 0.7...) si le plateau surcompense
const double BUTEE_DEG      = 50.0;     // débattement max ±
const double ZONE_MORTE_DEG = 0.3;      // pas de nouvelle impulsion si changement plus petit

// PID (sortie = vitesse de la consigne en °/s)
const double KP   = 8.0;    // 1/s   : réactivité (bande passante ~ 1,3 Hz) ; monter = plus vif
const double KI   = 0.5;    // 1/s²  : supprime l'écart résiduel ; garder petit
const double KD   = 0.15;   // s     : amortit, freine les dépassements
const double VMAX = 200.0;  // °/s   : vitesse max demandée au servo (ES09MD ~ 600 °/s à vide)

Servo servoX, servoY;

// =====================================================================
// 1) FILTRE COMPLÉMENTAIRE - fonction de Paul, même nom, valeurs corrigées
// =====================================================================
bool ItTest_EDTH_calculate( ItComponent_Data_t& ComponentData, int32_t
PeriodMillisecond, double& Pitch, double& Roll )
{
  static double PitchF = 0;
  static double RollF  = 0;
  static bool   Init   = false;

  double GyroSensitivity = 0.00875;
  double AccX  = ( double ) ComponentData._Acceleration1._x;
  double AccY  = ( double ) ComponentData._Acceleration1._y;
  double AccZ  = ( double ) ComponentData._Acceleration1._z + 1000;
  double GyroX = ( double ) ComponentData._Gyroscope1._x * GyroSensitivity;
  double GyroY = ( double ) ComponentData._Gyroscope1._y * GyroSensitivity;
// double Gyroz = ( double ) ComponentData._Gyroscope1._z * GyroSensitivity;
  double AccPitch = atan2( -AccX, sqrt( AccY * AccY + AccZ * AccZ ) ) *
180 / PI;
  double AccRoll  = atan2(  AccY, sqrt( AccX * AccX + AccZ * AccZ ) ) *
180 / PI;
// double AccRoll  = atan2(  AccY, AccZ ) * 180 / PI;
  double Factor = 0.98;
  double dt = PeriodMillisecond / 1000.0;

  if ( !Init ) { PitchF = AccPitch; RollF = AccRoll; Init = true; }

  PitchF = Factor * ( PitchF + GyroY * dt ) + ( 1 - Factor ) * AccPitch;
  RollF  = Factor * ( RollF  + GyroX * dt ) + ( 1 - Factor ) * AccRoll;

  Pitch = - PitchF + 11;
  Roll  =   RollF;

  return true;
}

// =====================================================================
// 2) PID d'un axe : fait suivre à la consigne servo l'angle à compenser
// =====================================================================
struct ItPid_t {
  double Cmd     = 0;   // consigne servo actuelle (°)
  double Integ   = 0;   // intégrale de l'erreur
  double ErrPrec = 0;   // erreur précédente
  double Envoyee = 0;   // dernière consigne réellement envoyée (zone morte)
};
ItPid_t PidX, PidY;

double ItTest_EDTH_pid( ItPid_t& P, double AngleDrone, int32_t PeriodMillisecond )
{
  double dt    = PeriodMillisecond / 1000.0;
  double Cible = GAIN * AngleDrone;                 // angle à compenser
  double Err   = Cible - P.Cmd;

  P.Integ += Err * dt;
  P.Integ  = constrain( P.Integ, -20.0, 20.0 );     // anti-emballement de l'intégrale
  double Deriv = ( dt > 0 ) ? ( Err - P.ErrPrec ) / dt : 0;
  P.ErrPrec = Err;

  double Vitesse = KP * Err + KI * P.Integ + KD * Deriv;   // °/s
  Vitesse = constrain( Vitesse, -VMAX, VMAX );
  P.Cmd  += Vitesse * dt;
  P.Cmd   = constrain( P.Cmd, -BUTEE_DEG, BUTEE_DEG );

  if ( fabs( P.Cmd - P.Envoyee ) >= ZONE_MORTE_DEG ) P.Envoyee = P.Cmd;
  return P.Envoyee;
}

int ItTest_EDTH_us( double AngleServo, double UsParDegre, int NeutreUs, int Sens )
{
  int Us = NeutreUs + ( int ) lround( Sens * AngleServo * UsParDegre );
  return constrain( Us, 1000, 2000 );
}

// =====================================================================
// SETUP / LOOP (adapter la lecture IMU à votre code)
// =====================================================================
void setup()
{
  Serial.begin( 115200 );
  servoX.attach( PIN_SERVO_X, 1000, 2000 );
  servoY.attach( PIN_SERVO_Y, 1000, 2000 );
  servoX.writeMicroseconds( NEUTRE_X_US );
  servoY.writeMicroseconds( NEUTRE_Y_US );
  delay( 1000 );
}

void loop()
{
  static uint32_t TPrec = millis();
  uint32_t T = millis();
  int32_t Periode = ( int32_t )( T - TPrec );
  if ( Periode < 10 ) return;                     // 100 Hz
  TPrec = T;

  ItComponent_Data_t ComponentData;
  // >>> ICI : votre lecture IMU qui remplit ComponentData <<<

  double Pitch, Roll;
  ItTest_EDTH_calculate( ComponentData, Periode, Pitch, Roll );

  double CmdX = ItTest_EDTH_pid( PidX, Roll,  Periode );
  double CmdY = ItTest_EDTH_pid( PidY, Pitch, Periode );
  servoX.writeMicroseconds( ItTest_EDTH_us( CmdX, US_PAR_DEGRE_X, NEUTRE_X_US, SENS_X ) );
  servoY.writeMicroseconds( ItTest_EDTH_us( CmdY, US_PAR_DEGRE_Y, NEUTRE_Y_US, SENS_Y ) );

  // Traces pour les graphiques : temps ; roulis drone ; tangage drone ; consigne X ; consigne Y
  Serial.print( T ); Serial.print( ';' );
  Serial.print( Roll, 2 ); Serial.print( ';' ); Serial.print( Pitch, 2 ); Serial.print( ';' );
  Serial.print( CmdX, 2 ); Serial.print( ';' ); Serial.println( CmdY, 2 );
}
