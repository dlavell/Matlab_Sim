#include "__cf_PackageDeliverySim.h"
#include <math.h>
#include "PackageDeliverySim_acc.h"
#include "PackageDeliverySim_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) { uint32_T lo ; uint32_T
hi ; lo = * u % 127773U * 16807U ; hi = * u / 127773U * 2836U ; if ( lo < hi
) { * u = 2147483647U - ( hi - lo ) ; } else { * u = lo - hi ; } return (
real_T ) * u * 4.6566128752457969E-10 ; } static void mdlOutputs ( SimStruct
* S , int_T tid ) { real_T * lastU ; real_T lastTime ; boolean_T p0nf2eyajo ;
idlexrjgpo * _rtB ; hbdftv5mfu * _rtP ; p14iuulsws * _rtX ; peas4oso0k *
_rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
p14iuulsws * ) ssGetContStates ( S ) ) ; _rtP = ( ( hbdftv5mfu * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ;
ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> ffqgknyjms
= ssGetT ( S ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> mjifqfvoci [ 0
] = _rtP -> P_0 [ 0 ] ; _rtB -> mjifqfvoci [ 1 ] = _rtP -> P_0 [ 1 ] ; _rtB
-> mjifqfvoci [ 2 ] = _rtP -> P_0 [ 2 ] ; memcpy ( & _rtB -> il2cowrefv [ 0 ]
, & _rtP -> P_1 [ 0 ] , 9U * sizeof ( real_T ) ) ; _rtB -> ng5fkojbve = _rtP
-> P_2 ; } ssCallAccelRunBlock ( S , 3 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit
( S , 1 , 0 ) ) { _rtB -> fjtiavwsww = _rtP -> P_3 ; } ssCallAccelRunBlock (
S , 1 , 0 , SS_CALL_MDL_OUTPUTS ) ; p0nf2eyajo = ( ( _rtB -> jf5cnzjkvo !=
0.0 ) && ( ! ( _rtB -> aypj1x4oja != 0.0 ) ) ) ; if ( ssIsSampleHit ( S , 1 ,
0 ) ) { _rtB -> c1rz5jh2km = ( _rtDW -> j4basf1zpu >= _rtP -> P_7 ) ; _rtB ->
mifbrflsws = _rtP -> P_8 ; } switch ( ( int32_T ) _rtB -> fjtiavwsww ) { case
1 : _rtB -> iuyukmc5cq = _rtB -> jf5cnzjkvo ; break ; case 2 : _rtB ->
iuyukmc5cq = p0nf2eyajo ; break ; case 3 : _rtB -> iuyukmc5cq = ( p0nf2eyajo
|| ( ( _rtB -> jf5cnzjkvo != 0.0 ) && ( _rtB -> aypj1x4oja != 0.0 ) && _rtB
-> c1rz5jh2km ) ) ; break ; default : _rtB -> iuyukmc5cq = _rtB -> mifbrflsws
; break ; } if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> hpukf20bqj = _rtP ->
P_9 ; } if ( ( _rtDW -> oarj5vv3y3 == ( rtMinusInf ) ) || ( _rtDW ->
oarj5vv3y3 == ssGetTaskTime ( S , 0 ) ) ) { _rtDW -> oarj5vv3y3 =
ssGetTaskTime ( S , 0 ) ; memcpy ( & _rtB -> fmjdgns3gz [ 0 ] , & _rtP ->
P_10 [ 0 ] , 19U * sizeof ( real_T ) ) ; } else { _rtB -> fmjdgns3gz [ 0 ] =
_rtX -> eeo550jiw0 [ 0 ] ; _rtB -> fmjdgns3gz [ 1 ] = _rtX -> eeo550jiw0 [ 1
] + _rtB -> hpukf20bqj ; memcpy ( & _rtB -> fmjdgns3gz [ 2 ] , & _rtB ->
c00lhnglkx [ 0 ] , 17U * sizeof ( real_T ) ) ; } if ( ssIsMajorTimeStep ( S )
) { ZCEventType zcEvent ; boolean_T resetIntg = false ; zcEvent = rt_ZCFcn (
ANY_ZERO_CROSSING , & ( ( ebtqtlmvbs * ) _ssGetPrevZCSigState ( S ) ) ->
d03ndqbhnu , _rtB -> iuyukmc5cq ) ; if ( _rtB -> iuyukmc5cq != 0.0 || _rtDW
-> jmgtnqu2of . IcNeedsLoading ) { resetIntg = true ; { int_T i1 ; real_T *
xc = & ( ( p14iuulsws * ) ssGetContStates ( S ) ) -> eeo550jiw0 [ 0 ] ; const
real_T * u2 = & _rtB -> fmjdgns3gz [ 0 ] ; for ( i1 = 0 ; i1 < 19 ; i1 ++ ) {
xc [ i1 ] = u2 [ i1 ] ; } } } else { if ( zcEvent ) resetIntg = true ; } if (
resetIntg ) { ssSetSolverNeedsReset ( S ) ; ssSetBlkStateChange ( S ) ; } } {
int_T i1 ; real_T * y0 = & _rtB -> pxcuu55oox [ 0 ] ; real_T * xc = & ( (
p14iuulsws * ) ssGetContStates ( S ) ) -> eeo550jiw0 [ 0 ] ; for ( i1 = 0 ;
i1 < 19 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } if ( ( _rtDW -> j4havqmcbz ==
( rtMinusInf ) ) || ( _rtDW -> j4havqmcbz == ssGetTaskTime ( S , 0 ) ) ) {
_rtDW -> j4havqmcbz = ssGetTaskTime ( S , 0 ) ; memcpy ( & _rtB -> g11lenrhvy
[ 0 ] , & _rtP -> P_11 [ 0 ] , 17U * sizeof ( real_T ) ) ; } else { memcpy (
& _rtB -> g11lenrhvy [ 0 ] , & _rtB -> pxcuu55oox [ 2 ] , 17U * sizeof (
real_T ) ) ; } if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> a550hsaijm = _rtP
-> P_13 * _rtP -> P_12 ; _rtB -> ga1xu011xa = _rtP -> P_14 * _rtP -> P_12 ;
_rtB -> gnqyrslfsv = _rtP -> P_15 * _rtP -> P_12 ; _rtB -> opcv0zkcog = _rtP
-> P_16 * _rtP -> P_12 ; _rtB -> akbf0wdixm = _rtP -> P_17 * _rtP -> P_12 ;
_rtB -> bmebxx2cq0 = _rtP -> P_18 * _rtP -> P_12 ; } ssCallAccelRunBlock ( S
, 2 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 30 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 31 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 32 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 33 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 34 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 35 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 36 ,
SS_CALL_MDL_OUTPUTS ) ; if ( ( _rtDW -> d2omkb40lq >= ssGetT ( S ) ) && (
_rtDW -> pjnqppg0zc >= ssGetT ( S ) ) ) { _rtB -> bts1k2bmf0 = 0.0 ; } else {
lastTime = _rtDW -> d2omkb40lq ; lastU = & _rtDW -> ia4c4krcdk ; if ( _rtDW
-> d2omkb40lq < _rtDW -> pjnqppg0zc ) { if ( _rtDW -> pjnqppg0zc < ssGetT ( S
) ) { lastTime = _rtDW -> pjnqppg0zc ; lastU = & _rtDW -> kckg1mt4pt ; } }
else { if ( _rtDW -> d2omkb40lq >= ssGetT ( S ) ) { lastTime = _rtDW ->
pjnqppg0zc ; lastU = & _rtDW -> kckg1mt4pt ; } } _rtB -> bts1k2bmf0 = ( _rtB
-> bvmjwbthkj - * lastU ) / ( ssGetT ( S ) - lastTime ) ; }
ssCallAccelRunBlock ( S , 5 , 38 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 39 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 40 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 41 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 42 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 43 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 44 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 45 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 46 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 47 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 48 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit
( S , 1 , 0 ) ) { _rtB -> k4ft0ph3mp = _rtP -> P_19 ; _rtB -> ggwiyztoud =
_rtP -> P_20 ; } _rtB -> ozkbkat0ze = ( ( _rtB -> pxcuu55oox [ 0 ] >= _rtB ->
k4ft0ph3mp ) || ( _rtB -> pxcuu55oox [ 1 ] >= _rtB -> ggwiyztoud ) || ( ! ( (
_rtB -> aypj1x4oja != 0.0 ) || ( _rtB -> jf5cnzjkvo != 0.0 ) ) ) ) ; if (
ssIsSampleHit ( S , 1 , 0 ) ) { if ( _rtB -> ozkbkat0ze ) {
ssSetStopRequested ( S , 1 ) ; } _rtB -> hksfm20b50 = _rtP -> P_21 ; _rtB ->
fg1cl2etno = _rtP -> P_22 ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { real_T * lastU ;
idlexrjgpo * _rtB ; hbdftv5mfu * _rtP ; p14iuulsws * _rtX ; peas4oso0k *
_rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
p14iuulsws * ) ssGetContStates ( S ) ) ; _rtP = ( ( hbdftv5mfu * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ;
ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_UPDATE ) ; if ( ssIsSampleHit (
S , 1 , 0 ) ) { _rtDW -> j4basf1zpu = ( _rtP -> P_5 - _rtP -> P_4 ) *
rt_urand_Upu32_Yd_f_pw_snf ( & _rtDW -> gznusywyhy ) + _rtP -> P_4 ; } _rtDW
-> jmgtnqu2of . IcNeedsLoading = 0 ; if ( _rtDW -> d2omkb40lq == ( rtInf ) )
{ _rtDW -> d2omkb40lq = ssGetT ( S ) ; lastU = & _rtDW -> ia4c4krcdk ; } else
if ( _rtDW -> pjnqppg0zc == ( rtInf ) ) { _rtDW -> pjnqppg0zc = ssGetT ( S )
; lastU = & _rtDW -> kckg1mt4pt ; } else if ( _rtDW -> d2omkb40lq < _rtDW ->
pjnqppg0zc ) { _rtDW -> d2omkb40lq = ssGetT ( S ) ; lastU = & _rtDW ->
ia4c4krcdk ; } else { _rtDW -> pjnqppg0zc = ssGetT ( S ) ; lastU = & _rtDW ->
kckg1mt4pt ; } * lastU = _rtB -> bvmjwbthkj ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { idlexrjgpo * _rtB ; p14iuulsws
* _rtX ; peas4oso0k * _rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S )
) ; _rtX = ( ( p14iuulsws * ) ssGetContStates ( S ) ) ; _rtB = ( ( idlexrjgpo
* ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 5 , 0 ,
SS_CALL_MDL_DERIVATIVES ) ; { if ( _rtB -> iuyukmc5cq == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 0 ] = _rtB -> hksfm20b50 ; }
else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 0 ] = 0.0 ; } if (
_rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 1 ] = _rtB -> fg1cl2etno ; } else { ( ( eng5ve0b2q * ) ssGetdX (
S ) ) -> eeo550jiw0 [ 1 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 2 ] = _rtB -> h0gw5goxcw [ 0 ]
; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 2 ] = 0.0 ; }
if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 3 ] = _rtB -> h0gw5goxcw [ 1 ] ; } else { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 3 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0
) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 4 ] = _rtB ->
h0gw5goxcw [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0
[ 4 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX
( S ) ) -> eeo550jiw0 [ 5 ] = _rtB -> gy005zvtb2 [ 0 ] ; } else { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 5 ] = 0.0 ; } if ( _rtB ->
iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 6 ]
= _rtB -> gy005zvtb2 [ 1 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 6 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 7 ] = _rtB -> gy005zvtb2 [ 2 ] ; } else {
( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 7 ] = 0.0 ; } if ( _rtB ->
iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 8 ]
= _rtB -> an4mvfc3ok [ 0 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 8 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 9 ] = _rtB -> an4mvfc3ok [ 1 ] ; } else {
( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 9 ] = 0.0 ; } if ( _rtB ->
iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 10 ]
= _rtB -> an4mvfc3ok [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 10 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 11 ] = _rtB -> j3b4qdjnfh [ 0 ] ; } else
{ ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 11 ] = 0.0 ; } if ( _rtB
-> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [
12 ] = _rtB -> j3b4qdjnfh [ 1 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) )
-> eeo550jiw0 [ 12 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 13 ] = _rtB -> j3b4qdjnfh [ 2
] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 13 ] = 0.0 ;
} if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 14 ] = _rtB -> pachzqrb0b ; } else { ( ( eng5ve0b2q * ) ssGetdX
( S ) ) -> eeo550jiw0 [ 14 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 15 ] = _rtB -> n5ghauyxs1 [ 0
] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 15 ] = 0.0 ;
} if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 16 ] = _rtB -> n5ghauyxs1 [ 1 ] ; } else { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 16 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0
) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 17 ] = _rtB ->
n5ghauyxs1 [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0
[ 17 ] = 0.0 ; } if ( _rtB -> iuyukmc5cq == 0.0 ) { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 18 ] = _rtB -> n5ghauyxs1 [ 3 ] ; } else { (
( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 18 ] = 0.0 ; } } } static
void mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 ,
652664912U ) ; ssSetChecksumVal ( S , 1 , 2858432438U ) ; ssSetChecksumVal (
S , 2 , 3878170160U ) ; ssSetChecksumVal ( S , 3 , 2119968884U ) ; { mxArray
* slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" )
; char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat ,
1 , & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat =
mxGetField ( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) {
status = 1 ; } else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; }
} mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
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
SysOutputFcn ) ( NULL ) ; } } static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
