`timescale 1us / 1us

module mips_tb;

reg reset, clock;

// The top level module name should be always "TopLevel"
PipelineCPU topLevel(.rst(reset), .clk(clock));

integer k;
initial begin
    // posedge clock

    // Hold reset for one cycle
    reset = 1;
    clock = 0; #1;
    clock = 1; #1;
    clock = 0; #1;
    reset = 0; #1;
    
    // This line is commented when testing
    // $stop;

    #1;
    for (k = 0; k < 60000; k = k + 1) begin
        clock = 1; #5;
        clock = 0; #5;
    end

    // Please finish with `syscall`, finishes here may mean the clocks are not enough (really?)
    $finish;
end
    
endmodule
