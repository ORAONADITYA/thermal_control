// -----------------------------------------------------------------------------
// fan_controller.v  -  three selectable fan strategies
//
//   mode 0 : fixed duty (default 50%)
//   mode 1 : on/off with hysteresis (+/- BAND around the setpoint)
//   mode 2 : proportional  duty = DUTY_MIN + kp * (T - T_SET), clamped 0..255,
//            with a safety override (100% fan) at or above T_LIMIT
//
// temp_q is Q16.16 degrees C. duty_cmd is combinational (0..255).
// The on/off hysteresis state is updated once per thermal tick.
//
// KP_Q is the proportional gain in duty-counts per degree C, Q8.8:
//   0.08 per C * 255 counts = 20.4 counts/C  ->  20.4 * 256 = 5222
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module fan_controller #(
    parameter signed [31:0] T_SET_Q    = 32'sd3604800,   // 55 C
    parameter signed [31:0] T_LIMIT_Q  = 32'sd4587520,   // 70 C
    parameter signed [31:0] BAND_Q     = 32'sd131072,    //  2 C
    parameter        [7:0]  FIXED_DUTY = 8'd128,         // ~50%
    parameter        [7:0]  DUTY_MIN   = 8'd38,          // ~15%
    parameter signed [15:0] KP_Q       = 16'sd5222
)(
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,
    input  wire        [1:0]  mode,
    input  wire signed [31:0] temp_q,
    output reg         [7:0]  duty_cmd
);

    // ---------------- on/off with hysteresis ----------------
    reg  on_reg;
    wire on_next = (temp_q > (T_SET_Q + BAND_Q)) ? 1'b1 :
                   (temp_q < (T_SET_Q - BAND_Q)) ? 1'b0 :
                   on_reg;

    always @(posedge clk) begin
        if (rst)
            on_reg <= 1'b0;
        else if (tick)
            on_reg <= on_next;
    end

    // ---------------- proportional ----------------
    wire signed [47:0] err_s      = temp_q - T_SET_Q;        // Q16.16
    wire signed [47:0] kp_s       = KP_Q;                    // sign-extended
    wire signed [47:0] dmin_s     = DUTY_MIN;                // zero-extended
    wire signed [47:0] prop_raw   = dmin_s + ((err_s * kp_s) >>> 24);

    wire        [7:0]  prop_duty  = (temp_q >= T_LIMIT_Q) ? 8'd255 :   // safety override
                                    (prop_raw < 0)        ? 8'd0   :
                                    (prop_raw > 255)      ? 8'd255 :
                                                            prop_raw[7:0];

    // ---------------- mode select ----------------
    always @(*) begin
        case (mode)
            2'd0:    duty_cmd = FIXED_DUTY;
            2'd1:    duty_cmd = on_next ? 8'd255 : 8'd0;
            default: duty_cmd = prop_duty;
        endcase
    end
endmodule
