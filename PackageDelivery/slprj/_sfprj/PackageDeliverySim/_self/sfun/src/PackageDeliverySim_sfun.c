/* Include files */

#include "PackageDeliverySim_sfun.h"
#include "PackageDeliverySim_sfun_debug_macros.h"
#include "c3_PackageDeliverySim.h"
#include "c4_PackageDeliverySim.h"
#include "c5_PackageDeliverySim.h"
#include "c6_PackageDeliverySim.h"
#include "c7_PackageDeliverySim.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _PackageDeliverySimMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void PackageDeliverySim_initializer(void)
{
}

void PackageDeliverySim_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_PackageDeliverySim_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==3) {
    c3_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

extern void sf_PackageDeliverySim_uses_exported_functions(int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[])
{
  plhs[0] = mxCreateLogicalScalar(0);
}

unsigned int sf_PackageDeliverySim_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1736393412U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(794808438U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2286465664U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(507275780U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1013805437U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3909147324U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2388721592U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1053152331U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 3:
        {
          extern void sf_c3_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c3_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c4_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c5_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c6_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c7_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3061339410U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1991824845U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3599338742U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2357874978U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3050212492U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1522582544U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2381116437U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3658684183U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_PackageDeliverySim_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(aiChksum, "OKh1fV4rRYXGeUpesI77pF") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "mbytYrrX0lBSQbhynMQHiG") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "Sf9vY1VlGktrG5PHu5MWmD") == 0) {
          extern mxArray *sf_c5_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c5_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "MaFZVuifaPa1gzfQyb4OcE") == 0) {
          extern mxArray *sf_c6_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c6_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "ykIM5Yk9MFP5SJJCVhf2FH") == 0) {
          extern mxArray *sf_c7_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c7_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_PackageDeliverySim_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        extern const mxArray
          *sf_c3_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_PackageDeliverySim_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(tpChksum, "jpgvuThrBTi9KpB6tuBhN") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "GEMjcO4joZO5zJDrbntEMH") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "LcyTGPNDeMw4735uYrZaCE") == 0) {
          extern mxArray *sf_c5_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c5_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "UQP0eh46WsC6ISr536buvD") == 0) {
          extern mxArray *sf_c6_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c6_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "Fpwh2NZcULebDHZY78PgiG") == 0) {
          extern mxArray *sf_c7_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c7_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_PackageDeliverySim_jit_fallback_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(tpChksum, "jpgvuThrBTi9KpB6tuBhN") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "GEMjcO4joZO5zJDrbntEMH") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "LcyTGPNDeMw4735uYrZaCE") == 0) {
          extern mxArray *sf_c5_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c5_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "UQP0eh46WsC6ISr536buvD") == 0) {
          extern mxArray *sf_c6_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c6_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "Fpwh2NZcULebDHZY78PgiG") == 0) {
          extern mxArray *sf_c7_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c7_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_PackageDeliverySim_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 3:
      {
        if (strcmp(tpChksum, "jpgvuThrBTi9KpB6tuBhN") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c3_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "GEMjcO4joZO5zJDrbntEMH") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c4_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "LcyTGPNDeMw4735uYrZaCE") == 0) {
          extern mxArray *sf_c5_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c5_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "UQP0eh46WsC6ISr536buvD") == 0) {
          extern mxArray *sf_c6_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c6_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "Fpwh2NZcULebDHZY78PgiG") == 0) {
          extern mxArray *sf_c7_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c7_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sf_PackageDeliverySim_get_post_codegen_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  unsigned int chartFileNumber = (unsigned int) mxGetScalar(prhs[0]);
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  switch (chartFileNumber) {
   case 3:
    {
      if (strcmp(tpChksum, "jpgvuThrBTi9KpB6tuBhN") == 0) {
        extern mxArray *sf_c3_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c3_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 4:
    {
      if (strcmp(tpChksum, "GEMjcO4joZO5zJDrbntEMH") == 0) {
        extern mxArray *sf_c4_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c4_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 5:
    {
      if (strcmp(tpChksum, "LcyTGPNDeMw4735uYrZaCE") == 0) {
        extern mxArray *sf_c5_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c5_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 6:
    {
      if (strcmp(tpChksum, "UQP0eh46WsC6ISr536buvD") == 0) {
        extern mxArray *sf_c6_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c6_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 7:
    {
      if (strcmp(tpChksum, "Fpwh2NZcULebDHZY78PgiG") == 0) {
        extern mxArray *sf_c7_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c7_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   default:
    break;
  }

  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void PackageDeliverySim_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _PackageDeliverySimMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "PackageDeliverySim","sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _PackageDeliverySimMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _PackageDeliverySimMachineNumber_,0);
}

void PackageDeliverySim_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_PackageDeliverySim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "PackageDeliverySim", "PackageDeliverySim");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_PackageDeliverySim_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
