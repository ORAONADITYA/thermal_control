// -----------------------------------------------------------------------------
// pwm_gen.v  -  8-bit PWM generator
// duty = 0..255  ->  0% .. 100% (255 is treated as always-on)
// One PWM period = 256 clock cycles.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module pwm_gen (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] duty,
    output reg        pwm_out
);
    reg [7:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            cnt     <= 8'd0;
            pwm_out <= 1'b0;
        end else begin
            cnt     <= cnt + 8'd1;
            pwm_out <= (duty == 8'd255) || (cnt < duty);
        end
    end
endmodule
