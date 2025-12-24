`include "csr_struct.vh"
module ExceptReg(
    input clk,
    input rst,
    input stall,
    input flush,
    input CsrPack::ExceptPack except_i,
    output CsrPack::ExceptPack except_o
);
    import CsrPack::ExceptPack;
    ExceptPack except;    

    always_ff @(posedge clk)begin
        if(rst|flush)begin
            except<='{except:1'b0,epc:64'b0,ecause:64'h0,etval:64'h0};
        end else if(~stall)begin
            except<=except_i;
        end
    end

    assign except_o=except;
    
endmodule
