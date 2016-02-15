#include "__cf_PackageDeliverySim.h"
#include <math.h>
#include "PackageDeliverySim_acc.h"
#include "PackageDeliverySim_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
static void mdlOutputs ( SimStruct * S , int_T tid ) { idlexrjgpo * _rtB ;
hbdftv5mfu * _rtP ; peas4oso0k * _rtDW ; _rtDW = ( ( peas4oso0k * )
ssGetRootDWork ( S ) ) ; _rtP = ( ( hbdftv5mfu * ) ssGetDefaultParam ( S ) )
; _rtB = ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ; if ( ssIsSampleHit ( S ,
1 , 0 ) ) { _rtB -> maph0qchgl [ 0 ] = _rtP -> P_0 [ 0 ] ; _rtB -> maph0qchgl
[ 1 ] = _rtP -> P_0 [ 1 ] ; _rtB -> maph0qchgl [ 2 ] = _rtP -> P_0 [ 2 ] ;
_rtB -> ebknc2x51p = _rtP -> P_1 ; _rtB -> busimgrrlu = _rtP -> P_2 ; _rtB ->
km3g4dovxr = _rtP -> P_3 ; _rtB -> exrocesjzz = _rtP -> P_4 ; _rtB ->
kyxgeqkhsb = _rtP -> P_5 ; ssCallAccelRunBlock ( S , 2 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; _rtB -> ifirm34nt0 = _rtP -> P_6 ;
ssCallAccelRunBlock ( S , 3 , 0 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> a4yme0r2it
[ 0 ] = _rtDW -> ogppga454i [ 0 ] ; _rtB -> a4yme0r2it [ 1 ] = _rtDW ->
ogppga454i [ 1 ] ; _rtB -> a4yme0r2it [ 2 ] = _rtDW -> ogppga454i [ 2 ] ;
ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ; if ( ( _rtB ->
j0kjnlbh43 != 0.0 ) && ( _rtP -> P_8 != 0.0 ) ) { ssSetStopRequested ( S , 1
) ; } ssCallAccelRunBlock ( S , 0 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 1 , 0 , SS_CALL_MDL_OUTPUTS ) ; } UNUSED_PARAMETER
( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { idlexrjgpo * _rtB ;
peas4oso0k * _rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S ) ) ; _rtB
= ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ; if ( ssIsSampleHit ( S , 1 , 0 )
) { _rtDW -> ogppga454i [ 0 ] = _rtB -> f1pupur3hn [ 0 ] ; _rtDW ->
ogppga454i [ 1 ] = _rtB -> f1pupur3hn [ 1 ] ; _rtDW -> ogppga454i [ 2 ] =
_rtB -> f1pupur3hn [ 2 ] ; } UNUSED_PARAMETER ( tid ) ; } static void
mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 , 2049211340U
) ; ssSetChecksumVal ( S , 1 , 688378095U ) ; ssSetChecksumVal ( S , 2 ,
1027229126U ) ; ssSetChecksumVal ( S , 3 , 1132604742U ) ; { mxArray *
slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" ) ;
char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat , 1
, & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat = mxGetField
( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) { status = 1 ; }
else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; } }
mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
status == 1 ) || ( strcmp ( slVerChar , "8.4" ) != 0 ) ) { return ; } }
ssSetOptions ( S , SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork (
S ) != sizeof ( peas4oso0k ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( idlexrjgpo ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
hbdftv5mfu ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & krmhsmofha ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ; } static
void mdlInitializeSampleTimes ( SimStruct * S ) { { SimStruct * childS ;
SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S , 0 ) ; callSysFcns
= ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 1 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 2 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 3 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 4 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; } } static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
