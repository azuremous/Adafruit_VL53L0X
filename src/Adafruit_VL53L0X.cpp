/*!
 * @file Adafruit_VL53L0X.cpp
 *
 * @mainpage Adafruit VL53L0X time-of-flight sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's VL53L0X driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit VL53L0X breakout: https://www.adafruit.com/product/3317
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_VL53L0X.h"

#define VERSION_REQUIRED_MAJOR  1 ///< Required sensor major version
#define VERSION_REQUIRED_MINOR  0 ///< Required sensor minor version
#define VERSION_REQUIRED_BUILD  1 ///< Required sensor build

#define STR_HELPER( x ) #x ///< a string helper
#define STR( x )        STR_HELPER(x) ///< string helper wrapper

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  i2c_addr Optional I2C address the sensor can be found on. Default is 0x29
    @param debug Optional debug flag. If true, debug information will print out via Serial.print during setup. Defaults to false.
    @param  i2c Optional I2C bus the sensor is located on. Default is Wire
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
boolean Adafruit_VL53L0X::begin(uint8_t i2c_addr, boolean debug, TwoWire *i2c) {
  int32_t   status_int;
  int32_t   init_done         = 0;

  uint32_t  refSpadCount;
  uint8_t   isApertureSpads;
  uint8_t   VhvSettings;
  uint8_t   PhaseCal;

  // Initialize Comms
  pMyDevice->I2cDevAddr      =  VL53L0X_I2C_ADDR;  // default
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;
  pMyDevice->i2c = i2c;

  pMyDevice->i2c->begin();     // VL53L0X_i2c_init();

  // unclear if this is even needed:
  if( VL53L0X_IMPLEMENTATION_VER_MAJOR != VERSION_REQUIRED_MAJOR ||
      VL53L0X_IMPLEMENTATION_VER_MINOR != VERSION_REQUIRED_MINOR ||
      VL53L0X_IMPLEMENTATION_VER_SUB != VERSION_REQUIRED_BUILD )  {
      if( debug ) {
          Serial.println( F( "Found " STR(VL53L0X_IMPLEMENTATION_VER_MAJOR) "." STR(VL53L0X_IMPLEMENTATION_VER_MINOR) "."  STR(VL53L0X_IMPLEMENTATION_VER_SUB) " rev " STR(VL53L0X_IMPLEMENTATION_VER_REVISION) ) );
          Serial.println( F( "Requires " STR(VERSION_REQUIRED_MAJOR) "." STR(VERSION_REQUIRED_MINOR) "." STR(VERSION_REQUIRED_BUILD) ) );
      }

      Status = VL53L0X_ERROR_NOT_SUPPORTED;

      return false;
  }

  Status = VL53L0X_DataInit( &MyDevice );         // Data initialization

  if (! setAddress(i2c_addr) ) {
    return false;
  }

  Status = VL53L0X_GetDeviceInfo( &MyDevice, &DeviceInfo );

  if( Status == VL53L0X_ERROR_NONE )  {
      if( debug ) {
         Serial.println( F( "VL53L0X Info:" ) );
         Serial.print( F( "Device Name: ")  ); Serial.print( DeviceInfo.Name );
         Serial.print( F( ", Type: " ) ); Serial.print( DeviceInfo.Type );
         Serial.print( F( ", ID: " ) ); Serial.println( DeviceInfo.ProductId );

         Serial.print( F( "Rev Major: " ) ); Serial.print( DeviceInfo.ProductRevisionMajor );
         Serial.print( F( ", Minor: " ) ); Serial.println( DeviceInfo.ProductRevisionMinor );
      }

      if( ( DeviceInfo.ProductRevisionMinor != 1 ) && ( DeviceInfo.ProductRevisionMinor != 1 ) ) {
          if( debug ) {
              Serial.print( F( "Error expected cut 1.1 but found " ) );
              Serial.print( DeviceInfo.ProductRevisionMajor );
              Serial.print( ',' );
              Serial.println( DeviceInfo.ProductRevisionMinor );
          }

          Status = VL53L0X_ERROR_NOT_SUPPORTED;
      }
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: StaticInit" ) );
      }

      Status = VL53L0X_StaticInit( pMyDevice ); // Device Initialization
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: PerformRefSpadManagement" ) );
      }

      Status = VL53L0X_PerformRefSpadManagement( pMyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization

      if( debug ) {
          Serial.print( F( "refSpadCount = " ) );
          Serial.print( refSpadCount );
          Serial.print( F( ", isApertureSpads = " ) );
          Serial.println( isApertureSpads );
      }
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      if( debug ) {
          Serial.println( F( "VL53L0X: PerformRefCalibration" ) );
      }

      Status = VL53L0X_PerformRefCalibration( pMyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
      if( debug ) {
          Serial.println( F( "VL53L0X: SetDeviceMode" ) );
      }

      Status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING );        // Setup in single ranging mode
  }

  // Enable/Disable Sigma and Signal check
  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
  }

  if( Status == VL53L0X_ERROR_NONE ) {
      Status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
  }
  useContinuous = false;

  if( Status == VL53L0X_ERROR_NONE ) {
      return true;
  } else {
      if( debug ) {
          Serial.print( F( "VL53L0X Error: " ) );
          Serial.println( Status );
      }

      return false;
  }
}

/**************************************************************************/
/*! 
    @brief  Change the I2C address of the sensor
    @param  newAddr the new address to set the sensor to
    @returns True if address was set successfully, False otherwise
*/
/**************************************************************************/
boolean Adafruit_VL53L0X::setAddress(uint8_t newAddr) {
  newAddr &= 0x7F;

  Status = VL53L0X_SetDeviceAddress(pMyDevice, newAddr * 2); // 7->8 bit

  delay(10);

  if( Status == VL53L0X_ERROR_NONE ) {
    pMyDevice->I2cDevAddr = newAddr;  // 7 bit addr
    return true;
  }
  return false;
}

boolean Adafruit_VL53L0X::setContinuous(VL53L0X_DeviceModes DeviceMode){
    useContinuous = true;
    Status = VL53L0X_SetDeviceMode(pMyDevice, DeviceMode);
    if( Status == VL53L0X_ERROR_NONE ) {
        if(startMeasurement()){
            return true;
        }
    }
    return false;        
}

boolean Adafruit_VL53L0X::setInterrupt(int min, int max, VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality, VL53L0X_InterruptPolarity Polarity){
    
    if(!setGpioConfig(DeviceMode, Functionality, Polarity)){
        return false;
    }

    if(DeviceMode == VL53L0X_DEVICEMODE_CONTINUOUS_RANGING || DeviceMode == VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING){
        FixPoint1616_t minThreashHold = (min * 65536.0);
        FixPoint1616_t maxThreashHold = (max * 65536.0);
        if(!setInterruptThresholds(minThreashHold, maxThreashHold, DeviceMode)){
            return false;
        }
        if(setContinuous(DeviceMode)){
            return true;
        }
    }
    return false;
}

boolean Adafruit_VL53L0X::clearInterruptMask(){
    Status = VL53L0X_ClearInterruptMask(pMyDevice, 0);
    if( Status == VL53L0X_ERROR_NONE ) {
      return true;
    }
    return false; 
}

boolean Adafruit_VL53L0X::startMeasurement(){
    Status = VL53L0X_StartMeasurement(pMyDevice);
    if( Status == VL53L0X_ERROR_NONE ) {
      return true;
    }
    return false; 
}

boolean Adafruit_VL53L0X::setInterruptThresholds(FixPoint1616_t ThresholdLow,
	FixPoint1616_t ThresholdHigh, VL53L0X_DeviceModes DeviceMode){
    Status = VL53L0X_SetInterruptThresholds(pMyDevice, DeviceMode, ThresholdLow, ThresholdHigh);
    if( Status == VL53L0X_ERROR_NONE ) {
      return true;
    }
    return false;  
}

boolean Adafruit_VL53L0X::setGpioConfig(VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality, VL53L0X_InterruptPolarity Polarity){
	    Status = VL53L0X_SetGpioConfig(pMyDevice, 0, DeviceMode, Functionality, Polarity);
        if( Status == VL53L0X_ERROR_NONE ) {
            return true;
        }
	
	    return false ;
}

int Adafruit_VL53L0X::getMeasurementResult(){
    int result = -1;
    VL53L0X_RangingMeasurementData_t measure;
    if(useContinuous){
        getContinuousRangingMeasurement(&measure);
    }else{
        getSingleRangingMeasurement(&measure);
    }

    if (measure.RangeStatus != 4) {
        result = measure.RangeMilliMeter;
    }

    return result;
}

/**************************************************************************/
/*! 
    @brief  get a ranging measurement from the device
    @param  RangingMeasurementData the pointer to the struct the data will be stored in
    @param debug Optional debug flag. If true debug information will print via Serial.print during execution. Defaults to false.
    @returns True if address was set successfully, False otherwise
*/
/**************************************************************************/
VL53L0X_Error Adafruit_VL53L0X::getSingleRangingMeasurement( VL53L0X_RangingMeasurementData_t *RangingMeasurementData, boolean debug )
{
    VL53L0X_Error   Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t  LimitCheckCurrent;


    /*
     *  Step  4 : Test ranging mode
     */

    if( Status == VL53L0X_ERROR_NONE ) {
        if( debug ) {
            Serial.println( F( "sVL53L0X: PerformSingleRangingMeasurement" ) );
        }
        Status = VL53L0X_PerformSingleRangingMeasurement( pMyDevice, RangingMeasurementData );

        if( debug ) {
            printRangeStatus( RangingMeasurementData );
        }

        if( debug ) {
            VL53L0X_GetLimitCheckCurrent( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent );

          	Serial.print( F( "RANGE IGNORE THRESHOLD: " ) );
          	Serial.println( (float)LimitCheckCurrent / 65536.0 );

          	Serial.print( F( "Measured distance: " ) );
          	Serial.println( RangingMeasurementData->RangeMilliMeter );
        }
    }

    return Status;
}

VL53L0X_Error Adafruit_VL53L0X::getContinuousRangingMeasurement(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    VL53L0X_Error   Status = VL53L0X_ERROR_NONE;
    uint8_t pMeasurementDataReady;
    Status = VL53L0X_GetMeasurementDataReady(pMyDevice, &pMeasurementDataReady);
    if (Status == VL53L0X_ERROR_NONE && pMeasurementDataReady == 0x01){
        Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
        if (Status == VL53L0X_ERROR_NONE){
            Status = VL53L0X_ClearInterruptMask(pMyDevice, 0);
        }
    }
    return Status;
}


/**************************************************************************/
/*! 
    @brief  print a ranging measurement out via Serial.print in a human-readable format
    @param pRangingMeasurementData a pointer to the ranging measurement data
*/
/**************************************************************************/
void Adafruit_VL53L0X::printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData )
{
    char buf[ VL53L0X_MAX_STRING_LENGTH ];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString( RangeStatus, buf );

    Serial.print( F("Range Status: " ) );
    Serial.print( RangeStatus );
    Serial.print( F( " : " ) );
    Serial.println( buf );

}
