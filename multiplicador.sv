// Control
//*******************************************
//*******************************************
// A es menos pero cambia a mas
// B es multi
// C es resultado (igual)
// D (eliminar extra)

module control (
    //entradas
    input logic clk,
    input logic col_0, 
    input logic col_1, 
    input logic col_2, 
    input logic col_3,

    //Salidas
    output logic [6:0] seg,
    output logic [3:0] an,
    input logic [3:0]fila
);

    //Variables temporales
    //-----------------
    logic rst;
    
    //Separador y BCD
    logic [15:0] binary_in; 
    logic[15:0] bcd_out;   
    
    //Rebote
    logic col0;
    logic col1;
    logic col2;
    logic col3;

    //Divisor de clk
    logic clk_div;


    //teclado
    logic [3:0]tecla_pre;
    logic menos;
    logic multiplicador;
    logic igual;
    logic [3:0] tecla;

    //contador de tecla
    logic [2:0] tecla_cont;

    //Display
    logic [15:0] result;
    logic [3:0] An;
    logic [6:0] Seg;

    //Almacenamiento
    logic almac1;
    logic almac2;
    logic [3:0] num1_dec1;
    logic [3:0] num1_dec2;
    logic [3:0] num2_dec1;
    logic [3:0] num2_dec2;
    logic [11:0] num_result1;
    logic [11:0] num_result2;
    
    //operacion
    logic [3:0] tecla_opera;

    //Multiplicador
    logic [7:0] num1;
    logic [7:0] num2;
    logic start;
    logic [15:0] resultado;
    logic done;


    //Suma
    logic sum;

    //Instancias
    ///*************

    

    anillo_ctdr insta_cont_anillo1 (
        .clk(clk),
        .rst(rst),
        .fila(fila)
    );

    rebote insta_reb_col0 (
        .clk(clk),
        .boton(col_0), 
        .boton_sal(col0)
    );

    
    rebote insta_reb_col1 (
        .clk(clk),
        .boton(col_1), 
        .boton_sal(col1)
    );
    
    rebote insta_reb_col2 (
        .clk(clk),
        .boton(col_2), 
        .boton_sal(col2)
    );
    
    rebote insta_reb_col3 (
        .clk(clk),
        .boton(col_3), 
        .boton_sal(col3)
    );
    
    cont_tecla inst_tecla_cont(
        .clk(clk),
        .rst(rst),
        .tecla_pre(tecla_pre),
        .tecla_cont(tecla_cont)
    );
    // 001 decenas num1
    // 010 unidades num1
    // 011 operacion
    // 100 decenas num2
    // 101 unidades num2
    // 110 resultado (C)


    almacenamiento inst_alma(
        .clk(clk),
        .rst(rst),
        .almac1(almac1), //variable 1 o 0
        .almac2(almac2),
        .num1_dec1(num1_dec1), 
        .num1_dec2(num1_dec2),  
        .num2_dec1(num2_dec1),   
        .num2_dec2(num2_dec2),   
        .num_result1(num_result1), 
        .num_result2(num_result2)
    );


    display inst_displ (
        .result(result),
        .clk(clk),
        .Seg(Seg),
        .anodes(An) 
    );

    multiplicador inst_mult(
        .A(num1), 
        .B(num2), 
        .clk(clk), 
        .start(start),   
        .resultado(resultado), 
        .done(done)
    );
    codificador_bcd inst_codificador(
        .clk(clk),
        .binary_in(binary_in),
        .bcd_out(bcd_out)
    );

    //SumaAri inst_sum(
    //    .clk(clk),
     //   .rst(rst),
      //  .num1(num1),
      //  .num2(num2),
      //  .sum(sum)  
    //);

    //FSM
    //******************* 

    typedef enum logic [2:0] { 
        E0, E1 , E2, E3, E4, E5
    } estado;

    estado estado_act, estado_sig;

        //Para el estado actual
    always_ff @(posedge clk or posedge rst)begin
        if (rst)begin
            estado_act <= E0; //estado de espera
        end 
        else begin
            estado_act <= estado_sig;
        end
    end


    //Logica de cada estado
    always_ff @(posedge clk) begin
        estado_sig = estado_act;
        //tecla_pre = 4'bxxxx;
        case(estado_act)
            E0: begin
                if(fila == 4'b0001) begin
                    if (col0 != 4'b0) tecla_pre = 4'b0001; //1
                    else if (col1 != 4'b0) tecla_pre = 4'b0010; //2
                    else if (col2 != 4'b0) tecla_pre = 4'b0011; //3
                    else if (col3 != 4'b0) tecla_pre = 4'b1010; //A
                    else estado_sig = E0;
                end
                else if(fila == 4'b0010) begin
                    if (col0 != 4'b0) tecla_pre = 4'b0100; //4
                    else if (col1 != 4'b0) tecla_pre = 4'b0101; //5
                    else if (col2 != 4'b0) tecla_pre = 4'b0110; //6
                    else if (col3 != 4'b0) tecla_pre = 4'b1011; //B
                    else estado_sig = E0;
                end
                else if(fila == 4'b0100) begin
                    if (col0 != 4'b0) tecla_pre = 4'b0111; //7
                    else if (col1 != 4'b0) tecla_pre = 4'b1000; //8
                    else if (col2 != 4'b0) tecla_pre = 4'b1001; //9
                    else if (col3 != 4'b0) tecla_pre = 4'b1100; //C
                    else estado_sig = E0;
                end
                else if(fila == 4'b1000) begin
                    if (col0 != 4'b0) tecla_pre = 4'b1110; //E
                    else if (col1 != 4'b0) tecla_pre = 4'b0000; //0
                    else if (col2 != 4'b0) tecla_pre = 4'b1111; //F //***
                    else if (col3 != 4'b0) tecla_pre = 4'b1101; //D
                    else estado_sig = E0;
                end
                else if (tecla_cont == 3'b001 || tecla_cont == 3'b100) estado_sig = E1; //Pasa al estado de decimal.
                else if (tecla_cont == 3'b010 || tecla_cont == 3'b101) estado_sig = E2; //Pasa al estado de unidades.
                else if (tecla_cont == 3'b011 || tecla_cont == 3'b110) estado_sig = E3; //Pasa al estado de operacion.
                else begin
                    estado_sig = E0; //Estado de espera.
                end
            end

            //Decenas
            E1: begin
                if (tecla_cont == 3'b001) begin
                        tecla <= tecla_pre;
                        num1_dec1 <= tecla;
                        result <= num1_dec1;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                end 
                else if (tecla_cont == 3'b100) begin 
                        tecla <= tecla_pre;
                        num2_dec1 <= tecla;
                        result <= num2_dec1;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                end 
                else begin 
                    estado_sig = E0;
                    end
            end
        
        // Unidades
            E2: begin
                if (tecla_cont == 3'b010) begin
                    tecla <= tecla_pre;
                        num1_dec2 <= tecla;
                        result <= num1_dec2;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                end 
                else if (tecla_cont == 3'b100) begin 
                    tecla <= tecla_pre;
                        num2_dec2 <= tecla_pre;
                        result <= num2_dec2;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                end 
                else begin 
                    estado_sig = E0;
                end 
            end

            E3: begin
                if (tecla_cont == 3'b011) begin
                    if (tecla_pre == 4'b1011) begin //Multiplicacion
                        tecla_opera = tecla_pre;
                        if (tecla_opera == tecla_pre) estado_sig = E0;
                        else begin
                            estado_sig = E0;
                        end
                    end
                    else if (tecla_pre == 4'b1010) begin //suma
                        tecla_opera = tecla_pre;
                        if (tecla_opera == tecla_pre) estado_sig = E0;
                        else begin
                            estado_sig = E0;
                        end
                    end
                    else begin
                        estado_sig = E0;
                    end
                end
                else if (tecla_cont == 3'b110) begin
                    if (tecla_pre == 4'b1100) begin
                        if (tecla_opera == 4'b1011) estado_sig = E4;
                        else if (tecla_opera == 4'b1010) estado_sig = E5;
                        else begin
                            estado_sig = E0;
                        end
                    end
                end
                else begin
                    estado_sig = E0;
                end
            end
            
            E4: begin
                if (tecla_cont == 3'b110 && tecla_opera == 4'b1011 ) begin 
                    start = 1'b1;
                    if (start == 1) begin
                        num1 <= num_result1;
                        num2 <= num_result2;
                    end
                    else if (done == 1) begin
                        binary_in <= resultado;
                        result <= bcd_out;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                    end 
                end 
                else if (tecla_opera != 4'b1010) estado_sig = E0;
                else begin
                    estado_sig = E0;
                end
            end
            
            default: estado_sig = E0;
            E5: begin
                if (tecla_opera == 4'b1010) begin
                    if (num1 != 4'b0 && num2 != 4'b0) begin
                        result = sum;
                        seg[0] = Seg[0];
                        seg[1] = Seg[1];
                        seg[2] = Seg[2];
                        seg[3] = Seg[3];
                        seg[4] = Seg[4];
                        seg[5] = Seg[5];
                        seg[6] = Seg[6];
                        an[0] = An[0];
                        an[1] = An[1];
                        an[2] = An[2];
                        an[3] = An[3]; 
                    end
                end
                else if (tecla_opera != 4'b1010) estado_sig = E0;
                else begin
                    estado_sig = E0;
                end
            end
            default: estado_sig = E0;
        endcase
    end
endmodule


//Contador de anillo filas
//******************************
//******************************
module anillo_ctdr #(parameter WIDTH = 4) //Se define el tamano del contador 4.
(
    input clk,
    input rst,
    output reg [WIDTH-1:0] fila
);

always_ff @(posedge clk) begin
    if (!rst)
        fila <=1; //Se inicializa encendiendo el primer bit.
    else begin
        fila[WIDTH-1] <= fila[0];
        for (int i = 0; i < WIDTH-1; i=i+1 ) begin  //Es un shifter
            fila[i] <= fila[i+1];
        end
    end
end
endmodule


//Detector de teclas con FSM.
//*************************************
//*************************************
module detector_columna (
    input logic clk,
    input logic rst,
    input logic [3:0]fila,  //entrada de contador de anillo;
    
    //entradas fisicas a FPGA
    input logic col_0, 
    input logic col_1, 
    input logic col_2, 
    input logic col_3, 

    output logic [3:0]tecla_pre,    //salida de teclas en bits.
    output logic menos,             //salida de codigo de signo menos.
    output logic multiplicador,       //salida de multiplicador. 
    output logic igual              //salida de codigo de igual.
);

    //se definen estados de la FSM
    typedef enum logic [4:0] { 
        F0, F1 , F2, F3, 
        F0C0, F0C1, F0C2, F0C3,
        F1C0, F1C1, F1C2, F1C3,
        F2C0, F2C1, F2C2, F2C3,
        F3C0, F3C1, F3C2, F3C3
    } estado;

    estado estado_act, estado_sig;
    logic [3:0] salida;

    //Para el estado actual
    always_ff @(posedge clk or posedge rst)begin
        if (rst)begin
            estado_act <= F0; //estado de espera
        end 
        else begin
            estado_act <= estado_sig;
        end
    end

    //Logica combinacional de la FSM, entre los estados
    always_comb begin
        estado_sig = estado_act; //Estado por defecto
        case(estado_act)

            //Primero verifica si la fila esta activa, si no pasa a la otra
            //Si esta activa, depende de cual col este activa, se da el siguiente estado
            F0: begin
                if(fila == 4'b0001) begin
                    if (col_0) estado_sig = F0C0; 
                    else if (col_1) estado_sig = F0C1;
                    else if (col_2) estado_sig = F0C2;
                    else if (col_3) estado_sig = F0C3;
                    else estado_sig = F1;
                end
                else begin 
                    estado_sig = F1;
                end
            end
            F1: begin
                if (fila == 4'b0010)begin
                    if (col_0) estado_sig = F1C0;
                    else if (col_1) estado_sig = F1C1;
                    else if (col_2) estado_sig = F1C2;
                    else if (col_3) estado_sig = F1C3;
                    else estado_sig = F2;
                end
                else begin 
                    estado_sig = F2;
                end
            end
            F2: begin
                if(fila == 4'b0100) begin
                    if (col_0) estado_sig = F2C0;
                    else if (col_1) estado_sig = F2C1;
                    else if (col_2) estado_sig = F2C2;
                    else if (col_3) estado_sig = F2C3;
                    else estado_sig = F3;
                end
                else begin 
                    estado_sig = F3;
                end
            end
            F3: begin
                if (fila == 4'b1000)begin
                    if (col_0) estado_sig = F3C0;
                    else if (col_1) estado_sig = F3C1;
                    else if (col_2) estado_sig = F3C2;
                    else if (col_3) estado_sig = F3C3;
                    else estado_sig = F0;
                end
                else begin 
                    estado_sig = F0;
                end
            end
            default: estado_sig = F0;
        endcase     
    end 

    //Detectar estado y asignar codigo binario al estado para saber las teclas en binario
    always_ff @(posedge clk or posedge rst) begin 
        if (rst)begin 
            salida <= 4'b0000;
        end
        else begin
            case (estado_act)
                F0C0: salida <= 4'b0000; //1
                F0C1: salida <= 4'b0001; //2
                F0C2: salida <= 4'b0010; //3
                F0C3: salida <= 4'b0011; //A  (menos)
                F1C0: salida <= 4'b0100; //4
                F1C1: salida <= 4'b0101; //5
                F1C2: salida <= 4'b0110; //6
                F1C3: salida <= 4'b0111; //B  (multiplicador)
                F2C0: salida <= 4'b1000; //7
                F2C1: salida <= 4'b1001; //8
                F2C2: salida <= 4'b1010; //9
                F2C3: salida <= 4'b1011; //C  (igual)
                F3C0: salida <= 4'b1100; //E
                F3C1: salida <= 4'b1101; //0
                F3C2: salida <= 4'b1110; //F
                F3C3: salida <= 4'b1111; //D
                default: salida <= 4'b0000;
            endcase
        end
    end

    //asigna la salida
    assign tecla_pre = salida;
    //se activa el menos

    always_comb begin
        menos = (salida == 4'b0011);
        multiplicador = (salida == 4'b0111);
        igual = (salida == 4'b1011);
    end

endmodule

// divisor de clk
//*******************************
//*******************************
module divisor (
    input logic clk,
    output reg clk_div
);

    parameter frecuencia = 27000000; //27 Mhz
    parameter fre = 1000000; //10hz
    parameter max_cuenta = frecuencia / (2*fre); //13.5 ciclos aprox 

    reg [4:0]cuenta;

    initial begin 
        cuenta = 0;
        clk_div = 0;
    end
    always_ff @(posedge clk) begin 
        if (cuenta == max_cuenta) begin //la cantidad de ciclos en alto o bajo
            clk_div <= ~clk_div;
            cuenta <= 0;
        end
        else begin
            cuenta <= cuenta+1;
        end
    end
endmodule


// Rebote mecanico
//****************************
//****************************
module rebote(
    input logic clk,
    input logic boton,
    output logic boton_sal
); 
    logic clk_hab;
    //Salidas de FF D, Q2_com es el complemento.
    logic q1, q2, q2_com, q0;

    divisor clk_ha( clk, clk_hab);

    FF_D_habilitador ff1(clk, clk_hab, boton, q0);
    FF_D_habilitador ff2(clk, clk_hab, q0, q1);
    FF_D_habilitador ff3(clk, clk_hab, q1, q2);

    assign q2_com = ~q2;
    assign boton_sal = q1 & q2_com; //AND para salida
endmodule


//FF_D se actualiza cuando clk_hab esta en alto.
module FF_D_habilitador(
    input logic clk, 
    input logic clk_hab,
    input logic D, 
    output reg Q=0
);
    always_ff @ (posedge clk) begin
        if(clk_hab == 1) 
            Q <= D;
    end
endmodule 


// Contador de tecla_pre
//****************************
//****************************

module cont_tecla (
    input logic clk,
    input logic rst,
    input logic [3:0] tecla_pre,

    output logic [2:0] tecla_cont
);

logic [3:0] tecla_ant;

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        tecla_ant <= 4'b0000;
        tecla_cont <= 3'b000;
    end
    else if (tecla_pre != tecla_ant) begin
        tecla_ant <= tecla_pre;
        tecla_cont <= tecla_cont + 1;
    end
end
endmodule


///Almacenamiento
//**********************************
//**********************************
module almacenamiento(
    input logic clk,
    input logic rst,
    input logic almac1,
    input logic almac2,
    input logic [3:0] num1_dec1,   // Primer dígito de 4 bits para numero1
    input logic [3:0] num1_dec2,   // Segundo dígito de 4 bits para numero1
    input logic [3:0] num2_dec1,   // Primer dígito de 4 bits para numero2
    input logic [3:0] num2_dec2,   // Segundo dígito de 4 bits para numero2
    output logic [11:0] num_result1,  // Resultado de la concatenación de num1 en decimal
    output logic [11:0] num_result2   // Resultado de la concatenación de num2 en decimal
);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Resetear los resultados a 0 cuando rst está activo
            num_result1 <= 12'b0;
            num_result2 <= 12'b0;
        end 
        else if (almac1) begin
            // Calcular los números concatenados cuando almac está activo
            num_result1 <= (num1_dec1 * 10) + num1_dec2; // Formar el número decimal num1
        end
         else if (almac2) begin
            // Calcular los números concatenados cuando almac está activo
            num_result1 <= (num1_dec1 * 10) + num1_dec2; // Formar el número decimal num2
         end 
    end

endmodule



//Idea 7-seg
//**************************************
//**************************************
`timescale 1ns / 1ps

module display (
    input logic [15:0] result,
    input logic clk,
    output logic [6:0] Seg,
    output logic [3:0] anodes
);

    logic [6:0] number;
    logic [3:0] state = 4'b0000; // Inicialización del estado
    logic [15:0] data;
    logic [3:0] dig;
    logic slow_clock = 0;
    integer count = 0;

    assign data = result;

    // Generador de reloj lento
    always_ff @(posedge clk) begin
        if (count > 2) begin// cambiar count > 100000 al probar la fpga
            count <= 0;
            slow_clock <= ~slow_clock;
        end else begin
            count <= count + 1;
        end
    end

    // Control de anodos y segmentos
    always_ff @(posedge slow_clock) begin
        case (state)
            4'b0000: begin
                anodes = 4'b1110;
                state = 4'b0001;
                dig = data[3:0];
            end
            4'b0001: begin
                anodes = 4'b1101;
                state = 4'b0010;
                dig = data[7:4];
            end
            4'b0010: begin
                anodes = 4'b1011;
                state = 4'b0011;
                dig = data[11:8];
            end
            4'b0011: begin
                anodes = 4'b0111;
                state = 4'b0000;
                dig = data[15:12];
            end
        endcase

        case (dig)
            4'b0000: number = 7'b1000000; // 0
            4'b0001: number = 7'b1111001; // 1
            4'b0010: number = 7'b0100100; // 2
            4'b0011: number = 7'b0110000; // 3
            4'b0100: number = 7'b0011001; // 4
            4'b0101: number = 7'b0010010; // 5
            4'b0110: number = 7'b0000010; // 6
            4'b0111: number = 7'b1111000; // 7
            4'b1000: number = 7'b0000000; // 8
            4'b1001: number = 7'b0011000; // 9
            4'b1010: number = 7'b0001000; // A
            4'b1011: number = 7'b0000011; // B
            4'b1100: number = 7'b1000110; // C
            4'b1101: number = 7'b0100001; // D
            4'b1110: number = 7'b0000110; // E
            4'b1111: number = 7'b0001110; // F
            default: number = 7'b0000000; // Apagado
        endcase
    end

    assign Seg = number;

endmodule


// Modulo de multiplicador
//*************************
//*************************
module multiplicador(
    input  logic [7:0] A, // Primer número de dos dígitos
    input  logic [7:0] B, // Segundo número de dos dígitos
    input  logic clk,     // Señal de reloj
    input  logic start,   // Señal para comenzar la multiplicación
    output logic [15:0] resultado, // Resultado de la multiplicación
    output logic done    // Indica cuando la multiplicación ha terminado
);
    logic [15:0] acumulador;
    logic [7:0] contador;
    logic [7:0] multiplicando;
    logic [7:0] multiplicador;
    logic busy;

    always_ff @(posedge clk) begin
        if (start) begin
            // Inicializamos los valores
            acumulador <= 16'd0;
            multiplicando <= A;
            multiplicador <= B;
            contador <= 8'd0;
            busy <= 1;
            done <= 0;
        end else if (busy) begin
            if (contador < multiplicador) begin
                acumulador <= acumulador + multiplicando;
                contador <= contador + 1;
            end else begin
                resultado <= acumulador;
                done <= 1;
                busy <= 0;
            end
        end
    end

endmodule


//Modulo del BCD
//****************************************
//****************************************
//Para dividir los numeros esteros
module codificador_bcd (
    input  logic        clk,          // Reloj de entrada
    input  logic [15:0] binary_in,    // Entrada binaria de 16 bits
    output logic [15:0] bcd_out       // Salida BCD de 16 bits
);
    // Variables internas
    logic [31:0] shift_reg;  // Registro de desplazamiento (16 bits BCD + 16 bits binarios)
    int state;               // Estado del contador para controlar las iteraciones

    always_ff @(posedge clk) begin
        if (state == 0) begin
            // Inicializar el registro de desplazamiento y contador
            shift_reg = {16'b0, binary_in};  // Concatenar 16 bits de ceros y la entrada binaria
            state = 1;  // Ir al primer estado
        end else if (state <= 16) begin
            // Algoritmo "Double Dabble" paso a paso
            for (int j = 0; j < 4; j++) begin
                if (shift_reg[19 + 4*j -: 4] >= 5) begin
                    shift_reg[19 + 4*j -: 4] += 3;
                end
            end
            shift_reg = shift_reg << 1;  // Desplazar a la izquierda
            state = state + 1;           // Incrementar estado
        end else begin
            // Asignar la salida BCD cuando el proceso termine
            bcd_out = shift_reg[31:16];
            state = 0;  // Reiniciar el estado para la próxima operación
        end
    end
endmodule


//suma
//**************************
//*************************

module SumaAri (
    input logic clk,           // Señal de reloj
    input logic rst,         // Señal de reset activa baja
    input logic [7:0] num1,   // Primer número de entrada (3 dígitos decimales)
    input logic [7:0] num2,   // Segundo número de entrada (3 dígitos decimales)
    output logic [8:0] sum    // Resultado de la suma (máximo 4 dígitos decimales)
);

    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            sum <= 8'd0; // Resetear el resultado de la suma
        end else begin
            sum <= num1 + num2; // Realizar la suma aritmética
        end
    end

endmodule
