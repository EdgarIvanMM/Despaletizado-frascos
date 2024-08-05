MODULE MainModule
    VAR num Camas:=0;
    VAR num Botella:=9;
    VAR num dz:=0;
    VAR num CamasTotalesPLC:=8;
    VAR num CamasTotales:=0;
    VAR num CamasFaltantes:=0;
    VAR num AlturaLibre:=0;
    var num AlturaBotella:=187.17;
    VAR num AlturaCarton:=1.524;
    VAR num AlturaMaxima:=0;
    VAR num LargoTarima:=1220;
    VAR num AnchoTarima:=1020;
    VAR num AltoTarima:=120.650;
    VAR num AnchoHerramienta:=1205.0;
    VAR num XoffsetBotellas:=0.0;
    VAR num YoffsetBotellas:=0.0;
    VAR num ZoffsetBotellas:=0.0;
    VAR num ZoffsetCarton:=0.0;
    VAR num Yoffset1:=0.0;
    VAR num Yoffset2:=0.0;
    VAR num Zoffset1:=0.0;
    VAR num Zoffset2:=0.0;
    VAR num Y1Tarima:=0.0;
    VAR num Y2Tarima:=0.0;
    VAR num Z1Tarima:=0.0;
    VAR num Z2Tarima:=0.0;
    
    PERS wobjdata wobj1:=[FALSE,TRUE,"",[[2161.14,959.553,-5.50794],[0.708509,0.00198023,0.00141332,0.705697]],[[0,0,0],[1,0,0,0]]];
    PERS wobjdata TarimaFrascos:=[FALSE,TRUE,"",[[1871.04,-909.046,-854.554],[0.999967,-0.00373754,0.00336609,-0.00634835]],[[0,0,0],[1,0,0,0]]];
    CONST robtarget HomeTrabajo:=[[527.18,-1065.92,2315.04],[0.00161823,-0.0183654,0.99983,0.000213364],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Banda:=[[-1202.99,-1517.32,1042.56],[0.00375807,-0.999987,-0.000228807,-0.00334316],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PreBanda:=[[-1222.33,-1434.81,1224.25],[0.00375805,-0.999987,-0.000235703,-0.00334318],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PosCero:=[[0,0,0],[0.00336722,-0.00664994,0.999965,0.00373653],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget ArribaBanda:=[[-1204.95,-1530.87,1345.51],[0.00375794,-0.999987,-0.000268444,-0.0033433],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget POS1BVIDRIOTEST:=[[-110.25,-1200.42,176.89],[0.00161531,-0.00530016,0.999985,0.00023449],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    TASK PERS loaddata load1:=[90,[1422.6,-1242,-1123],[1,0,0,0],0,0,0];
    CONST robtarget PuntoIntermedio2:=[[-941.91,-2199.30,1229.18],[0.00023282,-0.999979,-0.00633391,-0.00161555],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PosSegura:=[[246.17,-903.4,0],[0.000299876,-0.707849,-0.706346,-0.00502094],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget HOMEPOS:=[[586.29,45.91,2600],[0.00333264,0.00256585,0.999984,0.0037674],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PosSegura2:=[[-1157.39,-678.55,0],[0.00374565,-0.99998,-0.00393648,-0.00335707],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    TASK PERS loaddata BotellasPlastico:=[11.6,[1054.8,-213.3,-5338],[1,0,0,0],0,0,0];
    CONST robtarget DejarCartonFinal:=[[-2954.42,-1327.52,1225.22],[0.00375621,-0.999987,-0.000786,-0.00334525],[-2,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PreDejarCartonFinal:=[[-2957.04,-1330.48,1616.79],[0.00375621,-0.999987,-0.000786214,-0.00334525],[-2,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget SalidaDejarCartonFinal:=[[-1993.35,-1875.31,2300],[0.00305923,-0.981957,-0.189039,-0.00399261],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PosCeroBotellas:=[[-991.51,473.39,0],[0.00374569,-0.99998,-0.00392561,-0.00335703],[0,0,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PreDejarCartonFinal2:=[[-2661.01,-1329.24,1900],[0.00375621,-0.999987,-0.000784107,-0.00334524],[-1,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !Variables de velocidad.
    VAR speeddata vCarton:=[650, 500, 5000, 1000]; !Velocidad cuando se toma y deja el carton.
    VAR speeddata VEL_TEMP:=[800,50,1000,10]; 


    !PARAMETROS SENSORES
    !S1 ANCHO: P1 = 485 | P2 = 300
    !S2 LARGO: P1 = 485 | P2 = 300
    !SENSOR VACIO: N1 = -10.10 | N2 = -10.6
    
    PROC main()
        Reset RobotEnBanda;                     !Pone en 0 señal de "robot en banda"
        
        !Se toman los datos ingresados por operador en la receta cargada.
        TomarDatosPLC;                          !Llama a PROC TomarDatosPLC
        CamasTotales:=CamasTotalesPLC;          !Iguala nuestras variables del robot a las tomadas del PLC ingresadas por operador 
        CamasFaltantes:=CamasTotalesPLC;        ! = Linea 59 correspondiente a CamasTotales. 
        XoffsetBotellas:=LargoTarima/2+10;      ! = Linea 59. 
        YoffsetBotellas:=AnchoHerramienta/2-35; ! = Linea 59. 
        
        !Calcula altura inicial.
        AlturaMaxima:=AlturaBotella+AlturaCarton;           !Altura maxima se calcula sumando la altura de la botella + la altura del carton, antes ingresada en receta por operador.
        Despaletizado;                                      !Se llama a PROC Despaletizado.
        MoveJ HOMEPOS,v800,fine,tool1\WObj:=TarimaFrascos;  !Se posiciona en el HOME.
        PulseDO\PLength:=1,PLC_CICLO_TERMINADO_O;           !Estado como ciclo no terminado.
        stop;                                    
    ENDPROC

    !PROC que recibe los datos ingresados por operador.
    PROC TomarDatosPLC()
        CamasTotalesPLC:=GInput(PLC_CamasTotales);  !Toma las camas totales.
        AlturaBotella:=GInput(AltoBotella)/100.0;   !Toma la altura de la botella.
        AlturaCarton:=GInput(GrosorCarton)/1000.0;  !Toma la altura del carton.
        AltoTarima:=GInput(AlturaTarima)/100.0;     !Toma el alto de la tarima.
        LargoTarima:=GInput(LargooTarima);          !Toma largo de tarima.
        AnchoTarima:=GInput(AnchooTarima);          !Toma ancho de tarima.
        
        !Offsets que ingresa el operador desde HMI para acomodar ciertas posiciones mientras se ejecuta el ciclo.
        Yoffset1:=GInput(Y1Offsett);  
        Yoffset2:=GInput(Y2Offsett);
        Zoffset1:=GInput(Z1Offsett);
        Zoffset2:=GInput(Z2Offsett);
        Y1Tarima:=GInput(Y1Tarimaa);
        Y2Tarima:=GInput(Y2Tarimaa);
        Z1Tarima:=GInput(Z1Tarimaa);
        Z2Tarima:=GInput(Z2Tarimaa);
    ENDPROC

    !VERIFICA UTILIDAD DE PROC ChecarAltura.
    PROC ChecarAltura()
        ZoffsetBotellas:=AltoTarima+AlturaCarton+((AlturaMaxima)*(0));
        Movej Offs(PosCero,XoffsetBotellas,YoffsetBotellas,(ZoffsetBotellas+(AlturaBotella*2))),v400,fine,tool1\WObj:=TarimaFrascos;
        WaitTime 2;
        Movej Offs(PosCero,XoffsetBotellas,YoffsetBotellas-20,(ZoffsetBotellas+(AlturaBotella/2))),v200,z100,tool1\WObj:=TarimaFrascos;
        WaitTime 2;
        MoveL Offs(PosCero,XoffsetBotellas,YoffsetBotellas-10,ZoffsetBotellas+20),v200,fine,tool1\WObj:=TarimaFrascos;
        WaitTime 1;
        MoveL Offs(PosCero,XoffsetBotellas,YoffsetBotellas-2,ZoffsetBotellas-8),v200,fine,tool1\WObj:=TarimaFrascos;
        !bajarBotellas
        stop;
    ENDPROC

    !PROC principal para el desapletizado.
    PROC Despaletizado()
        GripLoad load0;
        
        !Ciclo para calcular alturas mientras se ejecuta el ciclo. Basandose en camas totales ingresadas por operador en receta.
        FOR i FROM CamasTotales TO 1 DO
            !Calcula altura de Botella
            ZoffsetBotellas:=AltoTarima+AlturaCarton+((AlturaMaxima)*(CamasTotales-1));
            AlturaLibre:=AltoTarima+AlturaCarton+((AlturaCarton+AlturaBotella)*CamasFaltantes);
            
            !POSICION INICIAL
            IF CamasTotalesPLC=CamasFaltantes THEN !Si camas totales es igual a camas faltantes quiere decir que se completaron (despaletizaron) todas las camas.
                MoveJ HOMEPOS,v1000,z200,tool1\WObj:=TarimaFrascos; !Se posiciona en HOME.
            ENDIF
            
            !Si camas totales y camas totalesPLC NO son iguales, aun no se completa el proceso, ENTONCES va por el carton siguiente y lo deja.
            IF CamasTotales<>CamasTotalesPLC THEN  
                TomarCarton;  !Llama a PROC que se encarga de tomar carton.
                DejarCartonn; !Llama a PROC que se encarga de dejar carton.
            ENDIF

            TomarBotellas; !Se llama a PROC que toma botellas.
            DejarBanda;    !Se llama a PROC que deja botellas.

            !Se verifica la altura de pallet. *PENDIENTE verificar importancia de saber el estado de 1200mm de altura*
            IF AlturaLibre>1200.0 THEN !Si la altura libre es mayor a 1200mm entonces esperar que la cama de la herramienta este dentro.
                WaitDI PLC_RetroCamaDentro,1;                                                   !Avisa que la cama este dentro.
                MoveJ offs(PosSegura2,0,0,AlturaLibre+150),v800,z150,tool1\WObj:=TarimaFrascos; !Se mueve a posicion segura. ()
            ELSE
                WaitDI PLC_RetroCamaDentro,1;                                           ! = Linea 131
                MoveJ offs(PosSegura2,0,0,1200.0),v800,z150,tool1\WObj:=TarimaFrascos;  ! = Linea 132
            ENDIF

            Decr CamasTotales;   !Se decrementa en 1 las camas totales para llevar estado de las camas que faltan en el proceso
            Decr CamasFaltantes; !=Linea 139
!           PLC_CamaActual := CamasFaltantes; !Mandar cama actual
        ENDFOR
    ENDPROC

    !PROC que toma botellas del pallet.
    PROC TomarBotellas()
        TomarDatosPLC;     !Se llama a proc para tomar datos del PLC.
        SeguridadAltura;   !Se llama a proc que se asegura de que no bajes tanto al tomar botellas *Verificar si aun es util 5/08/24*
        Set RobotEnBanda;  !Activa señal de robot en banda.
        

        IF CamasTotales=CamasTotalesPLC THEN  !Si las camas totales ingresadas por operador corresponden a camas totales de robot, entonces
            IF AlturaLibre>1200 THEN
                !Se mueve a la posicion antes de abrir herramienta (PosCero) para tomar botellas solo si la altura libre es MAYOR a 1200mm
                Movej Offs(PosCero,XoffsetBotellas,YoffsetBotellas,(ZoffsetBotellas+(AlturaBotella*2))),VEL_TEMP,z100,tool1\WObj:=TarimaFrascos; !POSICION antes de abrir herramineta para tomar botellas cuando el pallet mide mas de 1200mm.
                !Stop;
            ELSE
                !Se posiciona en la posicion antes de abrir herramienta (PosCero) para tomar botellas solo si la altura libre es MENOR a 1200mm
                Movej Offs(PosCero,XoffsetBotellas,YoffsetBotellas,(1350)),VEL_TEMP,z100,tool1\WObj:=TarimaFrascos; !POSICION antes de abrir herramineta para tomar botellas cuando el pallet mide menos de 1200mm.
                !Stop;
            ENDIF
        ELSE
            !Posicion cuando no coiniciden CamasTotales y CamasTotalesPLC *VERIFICAR EL POR QUE*
            Movej Offs(PosCero,XoffsetBotellas,YoffsetBotellas,(ZoffsetBotellas+(AlturaBotella*2))),VEL_TEMP,z100,tool1\WObj:=TarimaFrascos;
            !Stop;
        ENDIF

        SacarCama;  !PROC para sacar cama que toma botellas.
        WaitTime 1; !Espera un segundo
        AbrirGarra; !Abre garra. (Garras o palas que abrazan botellas)
        WaitTime 1; !Espera un segundo 

       !La ultima cama necesita una posicion diferente en cualquier modelo, por ello esta condicion:
       IF CamasFaltantes=1 THEN
            !MOVIMIENTO DENTRO DE PALLET, ULTIMA CAMA
            !Movej Offs(PosCero,XoffsetBotellas-20,YoffsetBotellas-40,(ZoffsetBotellas-20+(AlturaBotella/2))),v200,z100,tool1\WObj:=TarimaFrascos;
            Movej Offs(PosCero,XoffsetBotellas - 35,Y1Tarima + YoffsetBotellas - 20,(ZoffsetBotellas + (AlturaBotella / 2))),v200,z100,tool1\WObj:=TarimaFrascos;
            MoveL Offs(PosCero,XoffsetBotellas - 20,YoffsetBotellas - 40 + Y1Tarima,ZoffsetBotellas + 70 - Z1Tarima),v100,fine,tool1\WObj:=TarimaFrascos;
            Stop;
        ELSE
            !MOVIMIENTO DENTRO DE PALLET, RESTO DE CAMAS 
            Movej Offs(PosCero,XoffsetBotellas-35,YoffsetBotellas-20,(ZoffsetBotellas+(AlturaBotella/2))),v200,z100,tool1\WObj:=TarimaFrascos;
            MoveL Offs(PosCero,XoffsetBotellas-35,YoffsetBotellas+Yoffset2-20,ZoffsetBotellas+80-Zoffset2),v20,fine,tool1\WObj:=TarimaFrascos;
            Stop;
        ENDIF

        AcelModelo;     !Se llama a PROC con configuraciones de aceleraciones, *PENDIENTE verificar efectividad actual 05/08/24.
        GripLoad load1; !Verificar.
        WaitTime 1;     !Espera un segundo.
        CerrarGarra;    !Cierra garra (abraza botellas).
        Stop;  
        WaitTime 1;     !Espera un segundo.
        MeterCama;      !Mete la cama con los frascos abrazados.
        WaitTime 1;     !Espera un segundo.

        !Dependiendo altura de pallet hace el siguiente movimiento despues de tener frascos dento de la herramienta
        IF AlturaLibre>1200 THEN 
            MoveL Offs(PosCero,XoffsetBotellas,YoffsetBotellas-120,(ZoffsetBotellas+AlturaBotella+100)),v400,z100,tool1\WObj:=TarimaFrascos; !Posicion cuando pallet mide mas de 1200mm.
        ELSE
            MoveL Offs(PosCero,XoffsetBotellas,YoffsetBotellas-120,(1200)),v400,z100,tool1\WObj:=TarimaFrascos;                              !Posicion cuando pallet mide menos de 1200mm.
        ENDIF
        WaitTime 1;         !Espera un segundo
        Reset RobotEnBanda; !Pone en 0 señal de robot en banda
    ENDPROC

    !PROC que deja botellas en la banda
    PROC DejarBanda()
        !CALCULO ALTURA DE SALIDA DE CAMA EN HERRAMIENTA
        IF AlturaLibre>1200.0 THEN !Salida de cama en herramienta dependiendo altura de pallet.
            !Cuando el pallet mide mas de 1200 mm
            MoveJ offs(PosSegura,0,0,AlturaLibre+50),v800,z100,tool1\WObj:=TarimaFrascos;
            MoveJ offs(PosSegura2,0,0,AlturaLibre),v800,z100,tool1\WObj:=TarimaFrascos;
        ELSE
            !Cuando el pallet mide menos de 1200mm.
            MoveJ offs(PosSegura,0,0,1200.0),v800,z100,tool1\WObj:=TarimaFrascos;
            MoveJ offs(PosSegura2,0,0,1200.0),v800,z100,tool1\WObj:=TarimaFrascos;
        ENDIF

        WaitTime 1;                                             !Espera 1 segundo.
        MoveJ PreBanda,v400,z100,tool1\WObj:=TarimaFrascos;     !Se mueve a posicion preBanda. (Punto intermedio antes de llegar a posicion para dejar botellas, aqui se posiciona un poco mas arriba y alejado)
        WaitDI PLC_BANDA_ON,0;                                  !Espera señal de banda.     
        MoveJ Banda,v200,fine,tool1\WObj:=TarimaFrascos;        !Se mueve a posicion "Banda". (Punto donde deja los frascos, pegado a la banda listo para dejar frascos)
        WaitTime 1;                                             !Espera 1 segundo.
        PulseDO\PLength:=1,CamaFueraCompleta;                   !Saca la cama con frascos.
        WaitDI PLC_RetroCamaFuera,1;                            !Espera señal de que la cama esta afuera.
        PulseDO\PLength:=1,PLC_PARAR_BANDA;                     !
        WaitTime 2;                                             !Espera 2 segundos.
        AbrirGarra;                                             !Abre agarra que abraza los frascos.
        WaitTime 1;                                             !Espera 1 segundo.
        PathAccLim FALSE,FALSE;                                 !Quita las aceleraciones 
        GripLoad load0;                                         !
        MoveL ArribaBanda,v100,fine,tool1\WObj:=TarimaFrascos;  !Se mueve a punto arriba de la banda ya sin frascos (PENDIENTE verificar si se puede usar offset de punto BANDA)
        PulseDO\PLength:=1,PLC_CerrarGarra;                     !Se cierra la garra nuevamente ya que no tiene frascos y esta arriba de la banda
        WaitDI PLC_RetroGarraCerrada,1;                         !Epera señal de que la garra esta cerrada.     
        !WaitTime 1;
        PulseDO\PLength:=1,PLC_MeterCama;                       !Mete la cama.
        PulseDO\PLength:=1,PLC_ARRIBA_BOTELLAS_BANDA;           !Espera señal de que la cama esta dentro.
    ENDPROC

    !PROC que toma el carton del pallet.
    PROC TomarCarton()
        !Se calcula si el pallet mide mas de 1200mm.
        IF (ZoffsetBotellas+AltoTarima+AlturaBotella+AlturaCarton+100)>1200 THEN
            Movej Offs(PosCeroBotellas,0,0,(ZoffsetBotellas+AltoTarima+AlturaBotella+AlturaCarton+180)), vCarton, fine, tool1\WObj:=TarimaFrascos; !Posicion si el pallet mide mas de 1200mm. (Este es el movimiento que gira la herramienta antes de entrar por el carton)
        ELSE
            Movej Offs(PosCeroBotellas,0,0,(1200)), vCarton, fine, tool1\WObj:=TarimaFrascos; !Posicion si el pallet mide menos de 1200mm. (Este es el movimiento que gira la herramienta antes de entrar por el carton)
        ENDIF

        MoveL Offs(PosCeroBotellas,0,0,(ZoffsetBotellas + AltoTarima + AlturaBotella + AlturaCarton + 10)), vCarton, fine, tool1\WObj:=TarimaFrascos; !Punto donde agarra el carton.
        WaitRob\ZeroSpeed; !Espera que el robot este totalmente detenido. 
        ActivarVacio; !Activa el vacio para que las ventosas agarren el carton.
        WaitTime 1; !Espera un segundo.
        !Altura de salida del carton.
        Movej Offs(PosCeroBotellas,0,0,(ZoffsetBotellas + AltoTarima + AlturaBotella + AlturaCarton + 250)), vCarton, fine, tool1\WObj:=TarimaFrascos;
    ENDPROC

    !PROC que se encarga de abrir garras de la herramienta.
    PROC AbrirGarra()
        PulseDO\PLength:=1,PLC_AbrirGarra;
        !Señal para Abrir Garra
        WaitDI PLC_RetroGarraAbierta,1;
        RETURN ;
    ENDPROC

    !PROC que se encarga de cerrar garras de la herramienta.
    PROC CerrarGarra()
        PulseDO\PLength:=1,PLC_CerrarGarra;
        !Señal para Cerrar Garra
        WaitDI PLC_RetroGarraCerrada,1;
    ENDPROC

    !PROC que se encarga de sacar cama de la herramienta.
    PROC SacarCama()
        PulseDO\PLength:=1,PLC_SacarCama;
        !Señal para Sacar Cama
        WaitDI PLC_RetroCamaFuera,1;
        RETURN ;
    ENDPROC

    !PROC que se encarga de meter cama de la herramienta.
    PROC MeterCama()
        PulseDO\PLength:=1,PLC_MeterCama;
        WaitDI PLC_RetroCamaDentro,1;
    ENDPROC

    !PROC que se encarga de activar el vacio de la herramienta para ventosas.
    PROC ActivarVacio()
        PulseDO\PLength:=1,PLC_ACT_VACIO_O;
    ENDPROC

    !PROC que se encarga de desactivar vacio de la herramienta para ventosas.
    PROC DesVacio()
        PulseDO\PLength:=1,PLC_DESC_VACIO_O;
    ENDPROC

    !PROC que deja el carton.
    PROC DejarCartonn()
        !Dependiendo de altura de pallet. Si es menor o mayor a 1200mm.
        IF AlturaLibre>1200.0 THEN
            WaitDI PLC_RetroCamaDentro,1;                                                           !PENDIENTE, REVISAR POR QUE DEBE ESTAR CERRADA LA CAMA, RECORDAR QUE AL MOMENTO DE IR POR CARTON YA LLEVA LA CAMA DENTRO.
            MoveJ offs(PosSegura2,0,0,AlturaLibre+190), vCarton, z150, tool1\WObj:=TarimaFrascos;   !Va a posicion segura despues de tomar carton.
        ELSE
            WaitDI PLC_RetroCamaDentro,1;
            MoveJ offs(PosSegura2,0,0,1250.0), vCarton, z150, tool1\WObj:=TarimaFrascos;            !Va a posicion segura despues de tomar carton (Cuando el pallet mide menos de 1200mm).
        ENDIF
        
        !Existen tres puntos para el dejado de carton, preDejarCartonFinal2, preDejarCartonFinal, DejarCartonFinal. Cada uno de ellos va acercando al punto final hasta llegar al DejarCartonFinal que es la posicion final para dejar el carton.
        MoveJ PreDejarCartonFinal2, vCarton, z150, tool1\WObj:=TarimaFrascos;                       !Va a primera posicion intermedia antes de dejar carton.
        MoveJ PreDejarCartonFinal, vCarton, z150, tool1\WObj:=TarimaFrascos;                        !Va a segunda posicion intermedia antes de dejar carton.
        MoveL DejarCartonFinal, vCarton, z150, tool1\WObj:=TarimaFrascos;                           !Va a primera posicion final de dejar carton.
        WaitRob\ZeroSpeed;                                                                          !Esperar a robot detenido.
        DesVacio;                                                                                   !Desactiva vacio para soltar carton.
        WaitTime 1;                                                                                 !Esperar un segundo
        MoveJ PreDejarCartonFinal, vCarton, z150, tool1\WObj:=TarimaFrascos;                        !Va a segunda posicion intermedia antes de dejar carton, para salir de esa posicion.
        MoveJ SalidaDejarCartonFinal, vCarton, z150, tool1\WObj:=TarimaFrascos;                     !Punto para salir de dejar carton.

        !Si la altura libre es mayor a 1200. 
        IF AlturaLibre>1200.0 THEN
            !Este punto corresponde al movimiento que hace el robot arriba del pallet. 2/Agosto/24 se cambio posicion en Z ya que en frascos chicos no daba el giro sin golpearse contra el pallet.
            MoveJ offs(PosSegura,0,0,AlturaLibre+235), vCarton, z100, tool1\WObj:=TarimaFrascos; !Ir a posicion segura con altura correspondiente a altura libre mayor a 1200mm.
        ELSE
            !Este punto corresponde al movimiento que hace el robot arriba del pallet. 2/Agosto/24 se cambio posicion en Z ya que en frascos chicos no daba el giro sin golpearse contra el pallet.
            MoveJ offs(PosSegura,0,0,1255.0), vCarton, z100, tool1\WObj:=TarimaFrascos;          !Ir a posicion segura con altura correspondiente a altura libre menor a 1200mm.
        ENDIF
    ENDPROC

    !Actualmente no se utiliza este PROC. Pendiente en buscar funcion.
    PROC libre()
        FOR i FROM (CamasTotales) TO 0 DO
            AlturaLibre:=AltoTarima+GrosorCarton+((GrosorCarton+AltoBotella)*CamasFaltantes);
            ZoffsetBotellas:=AltoTarima+GrosorCarton+((GrosorCarton+AltoBotella)*Camas);
            ZoffsetCarton:=AltoTarima+GrosorCarton+((GrosorCarton+AltoBotella)*CamasFaltantes);
            IF CamasTotales=CamasFaltantes THEN
                MoveJ PosSegura,v800,z100,tool1\WObj:=TarimaFrascos;
            ENDIF

            IF AlturaLibre>1200.0 THEN
                MoveJ offs(PuntoIntermedio2,0,0,AlturaLibre+100), v400, z50, tool1\WObj:=TarimaFrascos;
            ELSE
                MoveJ offs(PuntoIntermedio2,0,0,1200.0), v400, z50, tool1\WObj:=TarimaFrascos;
            ENDIF
            WaitTime 1;

            IF CamasFaltantes<>CamasTotales THEN
                TomarCarton;
            ENDIF

            TomarBotellas;

            IF AlturaLibre>1200.0 THEN
                MoveJ offs(PuntoIntermedio2,0,0,AlturaLibre+100), v400, z50, tool1\WObj:=TarimaFrascos;
            ELSE
                MoveJ offs(PuntoIntermedio2,0,0,1200.0), v400, z50, tool1\WObj:=TarimaFrascos;
            ENDIF

            DejarBanda;

            IF CamasFaltantes<>CamasTotales THEN
                DejarCartonn;
            ENDIF

            Decr Camas;
            Decr CamasFaltantes;
        ENDFOR
    ENDPROC

    !Actualmente no se utiliza este PROC. Pendiente en buscar funcion.
    PROC ProbarTarima()
        Movej Offs(PosCero,XoffsetBotellas,-YoffsetBotellas,(ZoffsetBotellas+Ginput(AltoBotella)+100)),v100,fine,tool1\WObj:=TarimaFrascos;
        WaitTime 1;
        MoveL Offs(PosCero,XoffsetBotellas,-YoffsetBotellas,ZoffsetBotellas-5),v100,fine,tool1\WObj:=TarimaFrascos;
        !bajarBotellas
        WaitTime 1.5;
        MoveL Offs(PosCero,XoffsetBotellas,-YoffsetBotellas,(ZoffsetBotellas+Ginput(AltoBotella)+100)),v100,fine,tool1\WObj:=TarimaFrascos;
        WaitTime 1;
    ENDPROC

    !PROC modelo de aceleracion a usar.
    PROC AcelModelo()
        TEST Ginput(PLC_Pallet)
        CASE 3:
            GripLoad load1;
            PathAccLim TRUE\AccMax:=0.5,TRUE\DecelMax:=0.5;
        CASE 6:
            GripLoad load1;
            PathAccLim TRUE\AccMax:=0.5,TRUE\DecelMax:=0.5;
        CASE 9:
            PathAccLim TRUE\AccMax:=1,TRUE\DecelMax:=1;
            GripLoad load1;
        CASE 13:
            PathAccLim TRUE\AccMax:=0.2,TRUE\DecelMax:=0.5;
            GripLoad load1;
        CASE 14:
            PathAccLim TRUE\AccMax:=0.5,TRUE\DecelMax:=0.5;
            GripLoad load1;
        CASE 16:
            GripLoad load1;
            PathAccLim TRUE\AccMax:=0.5,TRUE\DecelMax:=0.5;
        CASE 34:
            PathAccLim TRUE\AccMax:=0.2,TRUE\DecelMax:=0.5;
            GripLoad load1;
        DEFAULT:
            RETURN ;
        ENDTEST
    ENDPROC

    !PROC de seguridad altura. No permite llegar al suelo.
    PROC SeguridadAltura()
        IF (ZoffsetBotellas-8-Z2Tarima)<108 THEN
            TPWrite "No puede bajar tanto";
            WaitTime 1;
            stop;
        ENDIF
    ENDPROC

ENDMODULE