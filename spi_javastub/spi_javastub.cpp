/*
* Copyright 2015 MICRORISC s.r.o.
* Copyright 2018 IQRF Tech s.r.o.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "spi_iqrf.h"
//#include "errors.h"
#include "com_microrisc_spi_SimpleSPI_Master.h"

#ifdef _DEBUG
#define DEBUG_TRC(msg) std::cout << std::endl << "{CPPDBG} " << __FUNCTION__ << ":  " << msg << std::endl;
#define PAR(par)                #par "=\"" << par << "\" "
#else
#define DEBUG_TRC(msg)
#define PAR(par)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
* Throws exception specified by its name.
* Copied from: The Java Native Interface. Programmer's Guide and Specification.
* @param name class name of the exception to be thrown
* @param msg message string of the exception
*/
static void JNU_ThrowByName(JNIEnv* env, const char* name, const char* msg)
{
  jclass cls = env->FindClass(name);
  // if cls is NULL, an exception has already been thrown
  if (cls != NULL) {
    env->ThrowNew(cls, msg);
  }
  // free the local ref
  env->DeleteLocalRef(cls);
}

/*
* Throws SPI Exception according to specified SPI error.
* Copied from: The Java Native Interface. Programmer's Guide and Specification.
* @param spiErr error object, which describes the SPI error
*/
//TODO
//static void JNU_ThrowSpiException(JNIEnv* env, errors_OperError* spiErr)
//{
//  jclass cls = env->FindClass("com/microrisc/rpi/spi/SPI_Exception");
//  // if cls is NULL, an exception has already been thrown
//  if (cls != NULL) {
//    if (spiErr != NULL) {
//      env->ThrowNew(cls, spiErr->descr);
//    } else {
//      env->ThrowNew(cls, "Exact cause was not found");
//    }
//  }
//  // free the local ref
//  env->DeleteLocalRef(cls);
//}

static void JNU_ThrowSpiException(JNIEnv* env, const char* spiErr)
{
  jclass cls = env->FindClass("com/microrisc/spi/SPI_Exception");
  // if cls is NULL, an exception has already been thrown
  if (cls != NULL) {
    if (spiErr != NULL) {
      env->ThrowNew(cls, spiErr);
    } else {
      env->ThrowNew(cls, "Exact cause was not found");
    }
  }
  // free the local ref
  env->DeleteLocalRef(cls);
}

/*
* SPI library initialization.
* @param masterId ID of SPI master
*/
void JNICALL Java_com_microrisc_spi_SimpleSPI_1Master_stub_1initialize
(JNIEnv* env, jobject jObj, jstring masterId) {
  DEBUG_TRC(__FUNCTION__)

  const char* masterIdUTF = env->GetStringUTFChars(masterId, NULL);

  if ( masterIdUTF == NULL ) {
    env->ReleaseStringUTFChars(masterId, masterIdUTF);
    jthrowable jException = env->ExceptionOccurred();
    if ( jException == NULL ) {
      JNU_ThrowByName(env, "java/lang/IllegalStateException", "GetStringUTFChars failed");
    }
    return;
  }

  // calling initialization function
  int operResult = spi_iqrf_init(masterIdUTF);
  env->ReleaseStringUTFChars(masterId, masterIdUTF);

  // if an error has occurred, throw exception
  //TODO
  if ( operResult != BASE_TYPES_OPER_OK ) {
    //errors_OperError* spiErr = rpi_spi_iqrf_getLastError();
    JNU_ThrowSpiException(env, "init failed");
  }
}

/*
* Gets SPI slave status.
*/
jobject JNICALL Java_com_microrisc_spi_SimpleSPI_1Master_stub_1getSlaveStatus
(JNIEnv* env, jobject jObj) {
  spi_iqrf_SPIStatus spiStatus = {
    0, SPI_IQRF_SPI_DISABLED
  };

  int operResult = spi_iqrf_getSPIStatus(&spiStatus);
  if ( operResult == BASE_TYPES_OPER_OK ) {
    jclass jStatClass = env->FindClass("com/microrisc/spi/SPI_Status");
    if ( jStatClass == NULL ) {
      JNU_ThrowByName(env, "com/microrisc/spi/SPI_Exception",
        "Cannot find com/microrisc/spi/iqrf/SPI_Status class"
        );
      return NULL;
    }

    jmethodID jStatConstructor = env->GetMethodID(jStatClass, "<init>", "(IZ)V");
    if ( jStatConstructor == NULL ) {
      JNU_ThrowByName(env, "com/microrisc/spi/SPI_Exception",
        "Cannot find the (IZ)V constructor for com/microrisc/spi/SPI_Status class"
        );
      return NULL;
    }

    jboolean jDataReady = ( spiStatus.isDataReady >= 1 )? JNI_TRUE : JNI_FALSE;
    jint jStatValue = ( spiStatus.isDataReady >= 1 )? spiStatus.dataReady : spiStatus.dataNotReadyStatus;
    jobject jSpiStatus = env->NewObject(
      jStatClass, jStatConstructor,jStatValue, jDataReady
      );
    return jSpiStatus;
  }

  //TODO
  //errors_OperError* spiErr = rpi_spi_iqrf_getLastError();
  JNU_ThrowSpiException(env, "getSlaveStatus failed");
  return NULL;
}


/*
* Writes data to SPI slave.
* @param dataToSend data to be sent
*/
void JNICALL Java_com_microrisc_spi_SimpleSPI_1Master_stub_1sendData
(JNIEnv* env, jobject jObj, jshortArray dataToSend) {
  jsize dataLen = env->GetArrayLength(dataToSend);
  if (dataLen > SPI_IQRF_MAX_DATA_LENGTH) {
    JNU_ThrowByName(env, "java/lang/IllegalArgumentException",
      "Length of data to send cannot be greather then SPI_IQRF_MAX_DATA_LENGTH"
      );
    return;
  }

  // getting pointer to data to be sent
  jshort* jArr = env->GetShortArrayElements(dataToSend, NULL);
  if (jArr == NULL) {
    JNU_ThrowByName(env, "java/lang/IllegalStateException",
      "GetShortArrayElements failed"
      );
    return;
  }

  // auxiliary buffer of data to be sent
  unsigned char* dataToWrite = (unsigned char*)malloc(dataLen * sizeof(unsigned char));
  if (dataToWrite == NULL) {
    JNU_ThrowByName(env, "java/lang/OutOfMemoryError",
      "Cannot allocate buffer for data to be sent"
      );
    return;
  }

  // copy the data into auxiliary buffer
  int i = 0;
  for (i = 0; i < dataLen; i++) {
    dataToWrite[i] = jArr[i] & 0xFF;
  }

  env->ReleaseShortArrayElements(dataToSend, jArr, 0);

  // send data to SPI slave
  int operResult = spi_iqrf_write(dataToWrite, dataLen);
  free(dataToWrite);

  //TODO
  if (operResult != BASE_TYPES_OPER_OK) {
    //errors_OperError* spiErr = rpi_spi_iqrf_getLastError();
    JNU_ThrowSpiException(env, "sendData failed");
  }
}


/*
* Reads data from SPI slave.
* @param dataLen length of data to read
*/
jshortArray JNICALL Java_com_microrisc_spi_SimpleSPI_1Master_stub_1readData
(JNIEnv* env, jobject jObj, jint dataLen) {
  if (dataLen > SPI_IQRF_MAX_DATA_LENGTH) {
    JNU_ThrowByName(env, "java/lang/IllegalArgumentException",
      "Length of data to read cannot be greather then SPI_IQRF_MAX_DATA_LENGTH"
      );
    return NULL;
  }

  // creating read buffer
  unsigned char* readBuffer = (unsigned char*)malloc(dataLen * sizeof(short));
  if (readBuffer == NULL) {
    JNU_ThrowByName(env, "java/lang/OutOfMemoryError", "Cannot allocate read buffer");
    return NULL;
  }

  // initializing read buffer to '0'
  memset(readBuffer, 0, dataLen);

  // reading data from SPI slave into buffer
  int operResult = spi_iqrf_read(readBuffer, dataLen);

  // checking, if some error occurred
  //TODO
  if (operResult != BASE_TYPES_OPER_OK) {
    //errors_OperError* spiErr = rpi_spi_iqrf_getLastError();
    free(readBuffer);
    //JNU_ThrowSpiException(env, spiErr);
    JNU_ThrowSpiException(env, "readData failed");
    return NULL;
  }

  // creating structure to return
  jshortArray readData = env->NewShortArray(dataLen);
  if (readData == NULL) {
    JNU_ThrowByName(env, "java/lang/IllegalStateException", "NewShortArray failed");
    return NULL;
  }

  // auxiliary buffer - for use in SetShortArrayRegion function
  jshort* jBuffer = (jshort*)malloc(dataLen * sizeof(jshort));
  if (jBuffer == NULL) {
    JNU_ThrowByName(env, "java/lang/OutOfMemoryError", "Cannot allocate auxiliary buffer");
    return NULL;
  }

  // initialization of auxiliary buffer to read data
  int i = 0;
  for (i = 0; i < dataLen; i++) {
    jBuffer[i] = readBuffer[i];
  }

  free(readBuffer);

  // setting structure to return
  env->SetShortArrayRegion(readData, 0, dataLen, jBuffer);

  free(jBuffer);

  if (env->ExceptionCheck()) {
    JNU_ThrowByName(env, "java/lang/IllegalStateException", "SetShortArrayRegion failed");
    return NULL;
  }

  return readData;
}

/*
* SPI library deinitialization and resources freeing.
*/
void JNICALL Java_com_microrisc_spi_SimpleSPI_1Master_stub_1destroy
(JNIEnv* env, jobject jObj) {
  int operResult = spi_iqrf_destroy();

  //TODO
  if (operResult != BASE_TYPES_OPER_OK) {
    //errors_OperError* spiErr = rpi_spi_iqrf_getLastError();
    //JNU_ThrowSpiException(env, spiErr);
    JNU_ThrowSpiException(env, "destroy failed");
  }
}

#ifdef __cplusplus
}
#endif
