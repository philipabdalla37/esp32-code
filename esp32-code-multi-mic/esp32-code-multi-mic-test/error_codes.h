/************************************************
*@file error_codes.h
*@created by Philip Abdalla
*@brief Error Codes
*/

#ifndef ERROR_CODES_
#define ERROR_CODES_

/***********************************************/
typedef enum
{
  CAPSTONE_SUCCESS = 0,
  CASPSTONE_FAIL,
  CAPSTONE_INVALID_INDEX,
  CAPSTONE_INVALID_PARAMETER,
  CAPSTONE_ERROR_NULL,
} capstoneErrorCode_t;

#endif /*ERROR_CODES_*/