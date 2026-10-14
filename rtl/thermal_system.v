// -----------------------------------------------------------------------------
// thermal_system.v  -  top level: controller + thermal plant + PWM + monitors
//
//   power_in --> thermal_model --> temp_q --> fan_controller --> duty --+
//                     ^                                                 |
//                     +-------------------------------------------------+
//                                                      duty_reg --> pwm_gen --> pwm_out
//
// Time scaling: one thermal step (0.5 s of "real" time) is simulated as
// TICK_DIV clock cycles. With TICK_DIV = 256 exactly one PWM period happens
// per thermal step, so the pwm_out waveform visibly shows the duty cycle.
//
// Outputs updated at each tick:
//   temp_q    : component temperature after the step (Q16.16 C)
//   duty_reg  : fan duty that was applied during that step (0..255)
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module thermal_system #(
    parameter TICK_DIV = 256,
    parameter signed [31:0] T_LIMIT_Q = 32'sd4587520   // 70 C
)(
    input  wire               clk,
    input  wire               rst,
    input  wire        [1:0]  mode,       // 0 fixed, 1 on/off, 2 proportional
    input  wire signed [31:0] power_in,   // component heat, Q16.16 W
    output wire signed [31:0] temp_q,
    output reg         [7:0]  duty_reg,
    output wire               pwm_out,
    output wire               over_limit,
    output reg         [39:0] energy_acc  // sum of duty^3 per step (see testbench for scaling)
);

    // ---------------- tick generator ----------------
    reg [$clog2(TICK_DIV)-1:0] tick_cnt;
    wire tick = (tick_cnt == TICK_DIV - 1);

    always @(posedge clk) begin
        if (rst)
            tick_cnt <= 0;
        else if (tick)
            tick_cnt <= 0;
        else
            tick_cnt <= tick_cnt + 1'b1;
    end

    // ---------------- controller ----------------
    wire [7:0] duty_cmd;

    fan_controller u_ctrl (
        .clk      (clk),
        .rst      (rst),
        .tick     (tick),
        .mode     (mode),
        .temp_q   (temp_q),
        .duty_cmd (duty_cmd)
    );

    // ---------------- plant ----------------
    thermal_model u_plant (
        .clk     (clk),
        .rst     (rst),
        .tick    (tick),
        .duty    (duty_cmd),
        .power_q (power_in),
        .temp_q  (temp_q)
    );

    // ---------------- registered duty, PWM, monitors ----------------
    wire [23:0] duty_cubed = duty_cmd * duty_cmd * duty_cmd;   // max 255^3 < 2^24

    always @(posedge clk) begin
        if (rst) begin
            duty_reg   <= 8'd0;
            energy_acc <= 40'd0;
        end else if (tick) begin
            duty_reg   <= duty_cmd;
            energy_acc <= energy_acc + duty_cubed;
        end
    end

    pwm_gen u_pwm (
        .clk     (clk),
        .rst     (rst),
        .duty    (duty_reg),
        .pwm_out (pwm_out)
    );

    assign over_limit = (temp_q >= T_LIMIT_Q);
endmodule
