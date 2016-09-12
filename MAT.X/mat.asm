;
; * machine à thé
; * Copyright 2016, Jacques Deschenes
; * 
; * This file is part of machineathe project.
; * 
; * ***  LICENCE ****
; * This program is free software; you can redistribute it and/or modify
; * it under the terms of the GNU General Public License as published by
; * the Free Software Foundation; either version 3 of the License, or
; * (at your option) any later version.
; * 
; * This program is distributed in the hope that it will be useful,
; * but WITHOUT ANY WARRANTY; without even the implied warranty of
; * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; * GNU General Public License for more details.
; * 
; * You should have received a copy of the GNU General Public License
; * along with this program; See 'copying.txt' in root directory of source.
; * If not, write to the Free Software Foundation,
; * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
; *  
; * to contact the author:  jd_temp@yahoo.fr
; * 
 ;
; MAT
; Machine à Thé.
; Description:  Machine avec minuterie pour le trempage d'un sachet de thé.
;   1 bouton pour réglé la durée en multiple de 15 secondes
;   1 bouton pour le démarrage de la séquence
;   1 servo moteur relié à un levier auquel est attaché le sachet. Le sachet est
;   descendu dans la tasse au début de la séquence et remonter à la fin.
;   1 affichage barre de DEL pour indiqué le temps.
;   1 haut-parleur pour l'arlame indiquant la fin de la séquence.
;
 
    include p16f1703.inc

;;;;;;;;;;;;;;
; config  bits
;;;;;;;;;;;;;;
    __config  _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF
    __config  _CONFIG2, _WRT_OFF & _PPS1WAY_ON & _ZCDDIS_OFF & _PLLEN_OFF & _STVREN_ON & _LPBOR_OFF & _LVP_OFF  
    
#define TONE_PIN RA1 ; sortie alarme    
#define SERVO_PIN RA2 ; sortie contrôle servo moteur  
#define TONE_LAT LATA
#define SERVO_LAT LATA    
#define TONE_TRIS TRISA
#define SERVO_TRIS TRISA
#define SERVO_WPU WPUA    
#define TONE_WPU WPUA    
#define TONE_PIN_PPS RA1PPS
#define TONE_OUT_FN  H'C' ; PWM1
#define TONE_CCPRL  CCPR1L
#define TONE_CCPCON  CCP1CON    
#define SERVO_PIN_PPS RA2PPS
#define SERVO_OUT_FN H'D'   ; PWM2
#define SERVO_CCPRL CCPR2L
#define SERVO_CCPCON CCP2CON    
#define START_PIN   RA5 ; bouton start sur RA5
#define START_TRIS TRISA
#define START_PORT PORTA    
#define TIMESET_PIN RA4 ; bouton time set sur RA4
#define TIMESET_TRIS TRISA    
#define TIMESET_PORT PORTA    
#define START_IOCN IOCAN ;registre détection transition négative
#define START_IOCF IOCAF ;registre indication interruption
#define BTN_TRIS TRISA 
#define BTN_PORT PORTA    
#define BTN_WPU  WPUA    
#define DISPLAY PORTC ; port barre de LED
#define DISP_TRIS TRISC
#define DISP_LAT  LATC
#define DISP_WPU  WPUC
    
; broches des anodes LED    
#define A1      RC0
#define A2	RC0
#define A3	RC0    
#define A4      RC1
#define A5	RC1
#define A6	RC1    
#define A7      RC2
#define A8	RC2
#define A9	RC2    
#define A10     RC3    
; broches des cathodes LED    
#define C1	RC5
#define C2	RC4
#define C3	RC3
#define C4	RC5
#define C5	RC4
#define C6	RC3
#define C7	RC5
#define C8	RC4
#define	C9	RC3
#define C10	RC2
;indicateurs booléens 
#define F_SEC    0 ; minuterie seconde 1=active, 0=inactive
#define F_MSEC4  1 ; minuterie msec4 1=active, 0=inactive    
    
#define DEBUG  
    
;;;;;;;;;;;;;;    
; constantes
;;;;;;;;;;;;;;
    constant F8MHZ=0xE ; valeur de OSCCON:IRCF pour 8Mhz
    constant F2MHZ=0xC ; valeur de OSSCCON:IRCF pour 2Mhz
    constant TIME_INCR = .15000/.4 ; incrément de 15 secondes
    constant TIME_MAX = TIME_INCR*.10 ; 150 secondes
    constant TMR0_DLY=.256-.250      ; compte pour TMR0
    constant TMR1_DLY=.65536-.32000  ; compte pour TMR1H:TMR1L
    constant SERVO_PERIOD=.155       ; compte pour PR2
    constant SERVO_POS_BAS=.53      ; compte pour CCP2RL:CCP2CON:DC2B
    constant SERVO_NEUTRAL_PULSE=.47 ; "
    constant SERVO_POS_HAUT=.59      ; "
    constant SERVO_DLY=.3  ; contrôle vitesse de rotation
;;;;;;;;;;;;;;    
; macros    
;;;;;;;;;;;;;;
; macros pour la gestion de la pile des arguments
pushw macro ; empile WREG  ( -- n )
    movwi ++FSR1
    endm
    
popw macro ; dépile dans WREG ( n -- )
    moviw FSR1--
    endm

over macro ; copie 2ième élément de la pile  ( n2 n1 - n2 n1 n2)
    moviw (-1)&0x3f[FSR1]
    pushw
    endm
 
dup macro ; copie TOS   ( n -- n n )
    moviw 0[FSR1]
    movwi ++FSR1
    endm
    
    
; toutes les pins du port DISP_PORT sont mises en mode entrée
; avec weak pullup actif
display_disable macro
    movlw 0x3f
    banksel DISP_TRIS
    movwf DISP_TRIS
    banksel DISP_WPU
    movwf DISP_WPU
    endm

; désactivation des weak pullup pour ne pas interférer avec
; le charlieplexing.
display_enable macro
    banksel DISP_WPU
    clrf DISP_WPU
    endm


; le canal PWM de l'alarme est désactivé.
; les broches TONE_PIN et ENV_PIN misent en mode entrée avec
; weak pullup
alarm_disable macro
    banksel TONE_TRIS ; tous les TRISx sont dans la même banque
    bsf TONE_TRIS, TONE_PIN
    bsf ENV_TRIS, ENV_PIN
    banksel TONE_WPU  ; tous les WPUx sont dans la même banque
    bsf TONE_WPU, TONE_PIN
    bsf ENV_WPU, ENV_PIN
    banksel TONE_CON
    bcf TONE_CON, TONE_OE ; désactive sortie PWM
    endm
    
;;;;;;;;;;;;;;
; variables
;;;;;;;;;;;;;;
    udata
stack res 16
 
    udata_shr
; 2 accumulateur 16 bits
ACAL res 1   ; accumulateur A octet poids faible
ACAH res 1  ; accumulateur A octet poids fort
ACBL res 1  ; accumulateur B  octet poids faible
ACBH res 1  ; accumulateur B  octet poids fort
trempage res 1 ; durée du trempage en secondes 
secondes res 1 ; compte à rebours secondes, interruption TIMER1
msec4 res 1 ; compte à rebours décrément 4 msec, interruption TIMER0
pwm_period res 1 ; période pour contrôler l'intensité du segment LED 
flags res 1; indicateurs booléens.
temp res 1 ; variable temporaire 
last_pos res 1 ; dernière commande envoyée au servo-moteur 
;;;;;;;;;;;;;;
; code
;;;;;;;;;;;;;;
    code 
    org 0
; vecteur reset    
rst:
    goto init
    nop
    nop
    nop
; service des interruptions    
    org 4
isr:
    btfss INTCON, T0IE
    bra test_tmr1
    btfsc INTCON, T0IF
    bra isr_timer0
test_tmr1:
    btfss INTCON, PEIE
    bra isr_exit ; interruption non traitée.
    banksel PIR1
    btfss PIR1, TMR1IF
    bra isr_exit ; interuption non traité
isr_timer1: ; répond à l'interuption sur TIMER1, intervalle 1 secondes
    bcf PIR1, TMR1IF
    banksel TMR1L
    movlw low TMR1_DLY
    movwf TMR1L
    movlw high TMR1_DLY
    movwf TMR1H
    decfsz secondes,F
    bra isr_exit
; minuterie expiré désactive interuption
    bcf flags, F_SEC
    banksel PIE1
    bcf PIE1,TMR1IE
    bcf INTCON, PEIE
    btfss INTCON, T0IE ; déscative toutes interruption
    banksel T1CON
    bcf T1CON, TMR1ON  ; arrête TIMER1
    bcf INTCON, GIE    ; si TIMER0 n'est pas en cours d'utilisation
    bra isr_exit
isr_timer0: ; répond à l'innterruption sur TIMER0, intervalle 4 msec.   
    banksel TMR0
    movlw TMR0_DLY
    movwf TMR0
    bcf INTCON,T0IF
    decfsz msec4,F
    bra isr_exit
disable_timer:    
    bcf flags, F_MSEC4
    bcf INTCON,T0IE
    btfss INTCON, PEIE
    bcf INTCON, GIE   ;si TIMER1 n'est pas utilisé présentiment
isr_exit:    
    retfie
    
init:
; fréquence clock à 8Mhz
    banksel OSCCON
    movlw F8MHZ<<IRCF0  ; Fosc 8Mhz
    movwf OSCCON
; initialisation pointeur de la pile des arguments
    movlw high stack
    movwf FSR1H
    movlw low stack
    movwf FSR1L
; assignation des périphériques au broches
; PWM1 sur TONE_PIN   
    banksel TONE_PIN_PPS
    movlw TONE_OUT_FN  ; fonction PWM1 
    movwf TONE_PIN_PPS   
; PWM2 sur SERVO_PIN
    banksel SERVO_PIN_PPS
    movlw SERVO_OUT_FN  ; fonction PWM2
    movwf SERVO_PIN_PPS
; veroullage PPS
    bsf PPSLOCK, 0
; activation low power sleep mode
    banksel VREGCON
    bsf VREGCON, VREGPM
; enlever weak pullup sur broche sortie TONE
    banksel TONE_WPU
    bcf TONE_WPU,TONE_PIN
; entrée analogique non utilisée
    banksel ANSELA
    clrf ANSELA
    clrf ANSELC
;    display_disable
; configuration OPTION_REG TIMER0 prescale 64
    banksel OPTION_REG
    movlw 4 ; TIMER0 préscale 1:32
    movwf OPTION_REG
; met SERVO_PIN en sortie bas voltage
    banksel SERVO_TRIS
    bcf SERVO_TRIS, SERVO_PIN
    banksel SERVO_LAT
    bcf SERVO_LAT, SERVO_PIN
;variables à zéro
    clrf flags 
    clrf secondes
    clrf msec4
    movlw SERVO_POS_HAUT
    movwf last_pos
    call self_test

main:
; activation interruption sur bouton START enfoncé
; ceci pour permettre la sortie du mode SLEEP en enfoncant ce bouton    
    banksel START_IOCF
    bcf START_IOCF, START_PIN ; remise à zéro indicateur interruption
    banksel START_IOCN
    bsf START_IOCN, START_PIN ;déclenchement interruption sur transition négative
    bsf INTCON, IOCIE  ; activation interruption sur transition broche.
    sleep
;désactivation IOCN
    bcf INTCON, IOCIE
    banksel START_IOCN
    bcf START_IOCN,START_PIN
; tonalité d'éveil
    movlw .20
    pushw
    movlw 1
    pushw
    call tone
; ajustement du temps
    call timeset
; descente du sachet dans la tasse
    movlw SERVO_POS_BAS 
    call servo_pos
;   attend expiration de la minuterie
;   avec mise à jour de LED_BAR    
    call timing_period
;   minuterie  expirée soulève le sachet    
    movlw SERVO_POS_HAUT
    call servo_pos
;   sonne l'alarme
    call alarm
; lorsque la séquence est terminée retourne en sommeil
    goto main
    
    
;l'utilisateur doit ajuster le temps de trempage    
timeset:
    clrf trempage
    banksel BTN_PORT
; attend qu'un bouton soit enfoncé.
timeset_wait_btn:   
    display_enable
    movlw .100
    call pause_msec
    movfw BTN_PORT
    andlw (1<<START_PIN)|(1<<TIMESET_PIN)
    xorlw (1<<START_PIN)|(1<<TIMESET_PIN)
    skpnz
    bra $-4
    btfsc WREG,TIMESET_PIN
    bra add_time
    btfsc WREG, START_PIN
    bra timeset_exit
    bra timeset_wait_btn
add_time:
    movlw .15
    addwf trempage
    movlw .151
    subwf trempage,W
    skpnc
    clrf trempage
    call beep
    movfw trempage
    call div15
    call light_segment
    bra timeset_wait_btn
timeset_exit:
    display_disable
    call beep
; attend relâchement du bouton    
    banksel BTN_PORT
    btfss BTN_PORT, START_PIN
    bra $-1
    return

; son court    
beep:
    movlw .10
    pushw
    clrw
    pushw
    call tone
    return
    
;  attend expiration de la minuterie
;  mise à jour affichage
#define RESTE ACAL    
timing_period:
    movfw trempage
    skpnz
    bra timeout
    call start_timer
    display_enable
wait_loop:
    banksel BTN_PORT
    btfss BTN_PORT, START_PIN ; cancellation si bouton enfoncé
    bra cancellation
    btfss flags, F_SEC
    bra timeout
    movfw secondes
    call div15
    movwf temp
    movf RESTE,F
    skpnz
    bra $+3
    incf temp
    bra $+3
    movlw 0xf
    movwf RESTE
; initialise période pour PWM intensité LED
    movlw 0xf
    movwf pwm_period
pwm_loop:
    movf RESTE,F
    skpnz
    clrf temp
    movfw temp
    call light_segment
    movf RESTE,F
    skpz
    decf RESTE,F
    decf pwm_period
    skpz
    bra pwm_loop
    bra wait_loop
cancellation:
    bcf flags, F_SEC
    bcf INTCON, T0IE
    clrf secondes
timeout:    
    display_disable
    return


; sonne l'alarme de fin de trempage    
alarm:
; initialition pointeur table CE3K
    movlw high CE3K
    movwf FSR0H
    bsf FSR0H,7
    movlw low CE3K
    movwf FSR0L
    moviw FSR0++
    pushw ;nombre de notes à jouer
;sauvegarde du pointeur
    movfw FSR0L
    movwf ACAL
    movfw FSR0H
    movwf ACAH
alarm_loop:
;restauration du pointeur    
    movfw ACAL
    movwf FSR0L
    movfw ACAH
    movwf FSR0H
    moviw FSR0++
    pushw  ; durée note
    moviw FSR0++
    pushw  ; indice note
;sauvegarde du pointeur    
    movfw FSR0L
    movwf ACAL
    movfw FSR0H
    movwf ACAH
    call tone
    decfsz INDF1
    bra alarm_loop
    popw
    return

; fait entendre une tonalité
; entrée:  pile ( d n -- )
;    d -> durée en multiple de 4 msec.
;    n -> indice de la note dans la table SCALE    
tone:
; prépare FSR0 pour accéder table SCALE
    movlw high SCALE
    movwf FSR0H
    bsf FSR0H,7
    movlw low SCALE
    movwf FSR0L
;ajoute l'indice de la note à FSR0    
    popw
    addwf FSR0L
    moviw 0[FSR0]
    banksel PR2
    movwf PR2  ;la valeur retirée de SCALE détermine la période PWM
    pushw	;sauvegarde sur la pile temporairement
    lsrf WREG   ; cette valeur divisé par 2 va dans CCPxL
    banksel TONE_CCPRL
    movwf TONE_CCPRL   ; les 6 bits les plus significatifs du rapport cyclique
    popw
    andlw 1    ; les 2 bites les moins significatifs du rapport cyclique
    lslf WREG
    swapf WREG ; dans bits 4 et 5 de CCPxCON
    iorlw (3<<CCP1M2) ; mode PWM
    movwf TONE_CCPCON  ; PWM configuré
; configuration de la sortie pour le PWM
    banksel TONE_TRIS
    bcf TONE_TRIS, TONE_PIN ; broche en mode sortie
; activation minuterie TIMER2
    banksel T2CON
    movlw (2<<T2CKPS0)|(1<<TMR2ON)
    movwf T2CON ; préscale 16 et TIMER2 on.
    popw     ; durée
    call pause_msec  
;désactive tone pwm
;arrête TIMER2
    banksel T2CON
    bcf T2CON, TMR2ON
    banksel TONE_CCPCON
    clrf TONE_CCPCON
    banksel TONE_TRIS
    bsf TONE_TRIS, TONE_PIN
    return
    
; allume un segment de la barre LED
; entrée:
;   W contient le numéro du segment
;   à allumer
light_segment:
    pushw
; initialise FSR0 pour pointer sur la table LED_CON
    movlw high LED_CONN
    movwf FSR0H
    bsf FSR0H, 7
    movlw low LED_CONN
    movwf FSR0L
    popw
    lslf WREG   ; segment * 2
    addwf FSR0L ; ajuste le pointeur de table
    movfw INDF0  ; obtient valeur sortie PORT
    banksel DISP_LAT
    movwf DISP_LAT
    incf FSR0L
    movfw INDF0  ; obtient valeur TRIS
    banksel DISP_TRIS
    movwf DISP_TRIS
    return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; commande la position du servo-moteur
; NOTE:
    ; la fréquence de l'oscillateur est descendue à 2Mhz pour
    ; permettre une période de 20msec avec T2
    ; le préscale de T2 est programmé à 1/64
    ; la période T=(PR2+1)*4*Tosc*PS=156*4*5e-7*64=0,019968
    ; la résolution est de Tosc*64=5e-7*64/4=32µSec
;  entrée:
;      W contient la valeur de contrôle
;  valeur comprise entre SERVO_POS_BAS et SERVO_POS_HAUT
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
servo_pos:
    pushw
; contrôle des limites    
    movlw SERVO_POS_BAS
    subwf INDF1,W
    movlw SERVO_POS_BAS
    skpc
    movwf INDF1
    movlw SERVO_POS_HAUT
    subwf INDF1,W
    movlw SERVO_POS_HAUT
    skpnc
    movwf INDF1
; pour obtenir une période de 50 Hertz sur TIMER2 il faut réduire
; Fosc à 2Mhz
    banksel OSCCON
    movlw F2MHZ<<IRCF0   ; Fosc=2Mhz
    movwf OSCCON
; configuration TIMER2 pour période 20 msec
    banksel T2CON
    movlw SERVO_PERIOD ; 20msec
    movwf PR2
    movlw (3<<T2CKPS0)|(1<<TMR2ON)
    movwf T2CON
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
; pour contrôler la vitesse de rotation
; on part de valeur de l'ancienne commande
; en progressant vers la nouvelle valeur    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
;durée du pulse 8 bits les plus significatifs dans CCP2RL et les 
;2 moins significatifs dans CCP2CON:4,5
servo_pos_loop:
    banksel SERVO_CCPCON
    lsrf last_pos,W
    lsrf WREG
    movwf SERVO_CCPRL
    swapf last_pos,W
    andlw (3<<4)
    iorlw 3<<CCP2M2  ; mode PWM
    movwf SERVO_CCPCON
; attend la fin du cycle
    movlw SERVO_DLY
    movwf ACAL
delay:
    banksel PIR1
    bcf PIR1,TMR2IF
    btfss PIR1,TMR2IF
    bra $-1
    decfsz ACAL,F
    bra delay
    movfw INDF1
    subwf last_pos,W
    skpnz
    bra servo_pos_exit
    movfw INDF1
    subwf last_pos,W
    skpc
    incf last_pos,F
    skpnc
    decf last_pos,F
    bra servo_pos_loop
servo_pos_exit:  
    popw
;arret de la commande
;attend la fin du cycle PWM
    banksel PR2
    movfw PR2
    subwf TMR2,W
    skpc
    bra $-3
    banksel SERVO_CCPCON
    clrf SERVO_CCPCON
    banksel T2CON
    bcf T2CON, TMR2ON
;remet Fosc à 8Mhz    
    banksel OSCCON
    movlw F8MHZ<<IRCF0  ; Fosc=8Mhz
    movwf OSCCON
    return
    
; division non signée entier 8 bits par 15
; entrée:
;   dividende dans W
;sortie:
;   quotient dans ACAH
;   reste dans ACAL    
#define DIVIDENDE ACAL    
#define QUOTIENT ACAH
#define DIVISEUR ACBL    
div15:
    clrf QUOTIENT
    xorlw 0
    skpnz
    bra div15_exit
    movwf DIVIDENDE
    movlw 0xF0 ; diviseur<<4
    movwf DIVISEUR
    subwf DIVIDENDE,W
    skpc
    bra div15_1
    bsf QUOTIENT,4
    movwf DIVIDENDE ; conserve reste pour prochaine étape
div15_1:
    lsrf DIVISEUR
    movfw DIVISEUR
    subwf DIVIDENDE,W
    skpc
    bra div15_2
    bsf QUOTIENT,3
    movwf DIVIDENDE
div15_2:
    lsrf DIVISEUR
    movfw DIVISEUR
    subwf DIVIDENDE,W
    skpc
    bra div15_3
    bsf QUOTIENT,2
    movwf DIVIDENDE
div15_3:
    lsrf DIVISEUR
    movfw DIVISEUR
    subwf DIVIDENDE,W
    skpc
    bra div15_4
    bsf QUOTIENT,1
    movwf DIVIDENDE
div15_4:
    lsrf DIVISEUR
    movfw DIVISEUR
    subwf DIVIDENDE,W
    skpnc
    bsf QUOTIENT,0
div15_exit:
    movfw QUOTIENT
    return

; démarre la minuterie des secondes
; utilisation du TIMER1 avec LFOSC.    
; entrée:
;   W=durée en secondes    
start_timer:
    movwf secondes
    banksel TMR1H
    movlw high TMR1_DLY
    movwf TMR1H
    movlw low TMR1_DLY
    movwf TMR1L
    movlw (3<<TMR1CS0)|1
    movwf T1CON
    banksel PIR1
    bcf PIR1, TMR1IF
    banksel PIE1
    bsf PIE1, TMR1IE
    bsf INTCON,PEIE
    bsf INTCON,GIE
    bsf flags, F_SEC
    return

    
; démarre la minuterie des millisecondes
; entrée:
;    W contient la durée en multiple de 4 msec.
start_msec4_tmr:
    movwf msec4
    banksel TMR0
    movlw TMR0_DLY
    movwf TMR0
    bcf INTCON, T0IF
    bsf INTCON, T0IE
    bsf INTCON, GIE
    bsf flags, F_MSEC4
    return

;pause en multiple de 4 msec
; entrée:  W -> pause= 4*W msec   
pause_msec:
    call start_msec4_tmr
    btfsc flags, F_MSEC4
    bra $-1
    return
    
;auto test à l'allumage
; allume tous les segments l'un après l'autre
; test alarme
; test servo-moteur
self_test:
    display_enable
    movlw .10
    pushw
disp_test:
    moviw 0[FSR1]
    call light_segment
    movlw .25
    pushw
    over
    call tone
    decfsz INDF1,F
    bra disp_test
    popw
    display_disable
tone_test:
    movlw .20
    pushw
    movlw .21
    pushw
    call tone
servo_test:
    movlw SERVO_POS_BAS
    call servo_pos
    movlw SERVO_POS_HAUT
    call servo_pos
    return
    
    
;;;;;;;;;;;;;;;;
;  tables
;;;;;;;;;;;;;;;;  
    org  0x700   
; connection des LED
; ordre inversé dans le montage final    
LED_CONN: ; PORT data, TRIS data
    dt 0, 0xFF ; tous les segments éteints
    dt 1<<A10, ~(1<<C10 | 1<<A10)
    dt 1<<A9, ~(1<<C9 | 1<<A9)
    dt 1<<A8, ~(1<<C8 | 1<<A8)
    dt 1<<A7, ~(1<<C7 | 1<<A7)
    dt 1<<A6, ~(1<<C6 | 1<<A6)
    dt 1<<A5, ~(1<<C5 | 1<<A5)
    dt 1<<A4, ~(1<<C4 | 1<<A4)
    dt 1<<A3, ~(1<<C3 | 1<<A3)
    dt 1<<A2, ~(1<<C2 | 1<<A2)
    dt 1<<A1, ~(1<<C1 | 1<<A1)
    dt 0, 0xFF
    dt 0, 0xFF
    dt 0, 0xFF
    dt 0, 0xFF
    dt 0, 0xFF

;valeur de PR2 pour généré tonalités gamme tempéré
SCALE:
    dt .238 ;0: 523 DO
    dt .225 ;1: 554 DO#
    dt .212 ;2: 587 RÉ
    dt .200 ;3: 622 RÉ#
    dt .189 ;4: 659 MI
    dt .178 ;5: 698 FA
    dt .168 ;6: 740 FA#
    dt .158 ;7: 784 SOL
    dt .149 ;8: 831 SOL#
    dt .141 ;9: 880 LA
    dt .133 ;10: 932 LA#
    dt .126 ;11: 988 SI
    dt .118 ;12: 1047 DO
    dt .112 ;13: 1109 DO#
    dt .105 ;14: 1175 RÉ
    dt .99  ;15: 1245 RÉ#
    dt .94  ;16: 1319 MI
    dt .88  ;17: 1397 FA
    dt .83  ;18: 1480 FA#
    dt .79  ;19: 1568 SOL
    dt .74  ;20: 1661 SOL#
    dt .70  ;21: 1760 LA
    dt .66  ;22: 1865 LA#
    dt .63  ;23: 1967 SI
    dt .59  ;24: 2093 DO
    
;5 première notes de rencontre du 3ième type
CE3K:
    dt .6 ; nombre de notes
    ; durée, note
    dt .100,.14
    dt .100,.16
    dt .100,.12
    dt .100,.0
    dt .200,.7
    dt .200,.7
    
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    end
    