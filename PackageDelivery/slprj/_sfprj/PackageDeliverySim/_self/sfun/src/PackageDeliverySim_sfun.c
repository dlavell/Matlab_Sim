/* Include files */

#include "PackageDeliverySim_sfun.h"
#include "PackageDeliverySim_sfun_debug_macros.h"
#include "c1_PackageDeliverySim.h"
#include "c2_PackageDeliverySim.h"
#include "c3_PackageDeliverySim.h"
#include "c4_PackageDeliverySim.h"

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
  if (chartFileNumber==1) {
    c1_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_PackageDeliverySim_method_dispatcher(simstructPtr, method, data);
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2415824755U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(396014895U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3568733914U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(537362215U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c1_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_PackageDeliverySim_get_check_sum(mxArray *plhs[]);
          sf_c2_PackageDeliverySim_get_check_sum(plhs);
          break;
        }

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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2229883471U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3222522108U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3152450654U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3455325069U);
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
     case 1:
      {
        if (strcmp(aiChksum, "oJgatzzzBpDsyQd4VY4S6E") == 0) {
          extern mxArray *sf_c1_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c1_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "71IfdMG65U32vM5upT99DB") == 0) {
          extern mxArray *sf_c2_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c2_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "kfWVSfO6kKGCk5D1gEfKVH") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "3gpruUj3IoKjVcC4hkEjqG") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_get_autoinheritance_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_get_autoinheritance_info();
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
     case 1:
      {
        extern const mxArray
          *sf_c1_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_PackageDeliverySim_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_PackageDeliverySim_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

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
     case 1:
      {
        if (strcmp(tpChksum, "PvdFqHDQPbnllc9ds4tPtB") == 0) {
          extern mxArray *sf_c1_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c1_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "zv1yPIO2t4kxmbm4GnkfWC") == 0) {
          extern mxArray *sf_c2_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c2_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "EkOprvxCoxR1qukCL3UY3E") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "d9QE5OuX4YINh34qm6EJgE") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_third_party_uses_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_third_party_uses_info();
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
     case 1:
      {
        if (strcmp(tpChksum, "PvdFqHDQPbnllc9ds4tPtB") == 0) {
          extern mxArray *sf_c1_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c1_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "zv1yPIO2t4kxmbm4GnkfWC") == 0) {
          extern mxArray *sf_c2_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c2_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "EkOprvxCoxR1qukCL3UY3E") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c3_PackageDeliverySim_jit_fallback_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "d9QE5OuX4YINh34qm6EJgE") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_jit_fallback_info(void);
          plhs[0] = sf_c4_PackageDeliverySim_jit_fallback_info();
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
     case 1:
      {
        if (strcmp(tpChksum, "PvdFqHDQPbnllc9ds4tPtB") == 0) {
          extern mxArray *sf_c1_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c1_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "zv1yPIO2t4kxmbm4GnkfWC") == 0) {
          extern mxArray *sf_c2_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c2_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "EkOprvxCoxR1qukCL3UY3E") == 0) {
          extern mxArray *sf_c3_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c3_PackageDeliverySim_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "d9QE5OuX4YINh34qm6EJgE") == 0) {
          extern mxArray *sf_c4_PackageDeliverySim_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c4_PackageDeliverySim_updateBuildInfo_args_info();
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
   case 1:
    {
      if (strcmp(tpChksum, "PvdFqHDQPbnllc9ds4tPtB") == 0) {
        extern mxArray *sf_c1_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c1_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 2:
    {
      if (strcmp(tpChksum, "zv1yPIO2t4kxmbm4GnkfWC") == 0) {
        extern mxArray *sf_c2_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c2_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 3:
    {
      if (strcmp(tpChksum, "EkOprvxCoxR1qukCL3UY3E") == 0) {
        extern mxArray *sf_c3_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c3_PackageDeliverySim_get_post_codegen_info();
        return;
      }
    }
    break;

   case 4:
    {
      if (strcmp(tpChksum, "d9QE5OuX4YINh34qm6EJgE") == 0) {
        extern mxArray *sf_c4_PackageDeliverySim_get_post_codegen_info(void);
        plhs[0] = sf_c4_PackageDeliverySim_get_post_codegen_info();
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
    "PackageDeliverySim","sfun",0,4,0,0,0);
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
