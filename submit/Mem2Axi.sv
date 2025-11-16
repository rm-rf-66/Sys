`include"mem_ift.vh"
module Mem2Axi (
    input logic clk,
    input logic rstn,
    Mem_ift.Slave mem_ift,
    Axi_ift.Master axi_ift
);
    initial begin
        assert(axi_ift.DATA_WIDTH == mem_ift.DATA_WIDTH) 
        else begin
            $display("the data width of axi_ift and mem_ift are different");
            $finish;
        end

        assert(axi_ift.ADDR_WIDTH == mem_ift.ADDR_WIDTH) 
        else begin
            $display("the addr width of axi_ift and mem_ift are different");
            $finish;
        end
    end
    assign axi_ift.r_request_bits.raddr = mem_ift.r_request_bits.raddr;
    assign axi_ift.r_request_valid = mem_ift.r_request_valid;
    assign mem_ift.r_request_ready =axi_ift.r_request_ready;

    assign mem_ift.r_reply_bits.rdata = axi_ift.r_reply_bits.rdata;
    assign mem_ift.r_reply_bits.rresp = axi_ift.r_reply_bits.rresp;
    assign mem_ift.r_reply_valid = axi_ift.r_reply_valid;
    assign axi_ift.r_reply_ready = mem_ift.r_reply_ready;

    assign mem_ift.w_reply_bits.bresp = axi_ift.w_reply_bits.bresp;
    assign mem_ift.w_reply_valid = axi_ift.w_reply_valid;
    assign axi_ift.w_reply_ready = mem_ift.w_reply_ready;

    logic [mem_ift.ADDR_WIDTH-1:0] waddr_data;
    logic waddr_valid;
    logic [mem_ift.DATA_WIDTH-1:0] wdata_data;
    logic [mem_ift.DATA_WIDTH/8-1:0] wmask_data;
    logic wdata_valid;

    wire w_request_proxy;
    assign w_request_proxy = waddr_valid | wdata_valid;
    assign mem_ift.w_request_ready = ~w_request_proxy;
    assign axi_ift.w_addr_request_bits.waddr = w_request_proxy ? waddr_data : mem_ift.w_request_bits.waddr;
    assign axi_ift.w_addr_request_valid = w_request_proxy ? waddr_valid : mem_ift.w_request_valid;
    assign axi_ift.w_data_request_bits.wdata = w_request_proxy ? wdata_data : mem_ift.w_request_bits.wdata;
    assign axi_ift.w_data_request_bits.wstrb = w_request_proxy ? wmask_data : mem_ift.w_request_bits.wmask;
    assign axi_ift.w_data_request_valid = w_request_proxy ? wdata_valid : mem_ift.w_request_valid;

    always_ff@(posedge clk)begin
        if(~rstn)begin
            waddr_data <= {mem_ift.ADDR_WIDTH{1'b0}};
            waddr_valid <= 1'b0;
        end else if(w_request_proxy)begin
            if(axi_ift.w_addr_request_ready)begin
                waddr_valid <= 1'b0;
            end
        end else if(mem_ift.w_request_valid & ~axi_ift.w_addr_request_ready)begin
            waddr_data <= mem_ift.w_request_bits.waddr;
            waddr_valid <= 1'b1;
        end
    end

    always_ff@(posedge clk)begin
        if(~rstn)begin
            wdata_data <= {mem_ift.DATA_WIDTH{1'b0}};
            wmask_data <= {mem_ift.DATA_WIDTH/8{1'b0}};
            wdata_valid <= 1'b0;
        end else if(w_request_proxy)begin
            if(axi_ift.w_data_request_ready)begin
                wdata_valid <= 1'b0;
            end
        end else if(mem_ift.w_request_valid & ~axi_ift.w_data_request_ready)begin
            wdata_data <= mem_ift.w_request_bits.wdata;
            wmask_data <= mem_ift.w_request_bits.wmask;
            wdata_valid <= 1'b1;
        end
    end
endmodule