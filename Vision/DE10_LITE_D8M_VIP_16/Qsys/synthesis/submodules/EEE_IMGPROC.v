module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// hsv
	hsv_h,
	hsv_s,
	hsv_v,
	
	//Area of aliens
	red_area,
	yellow_area,
	magenta_area,
	teal_area,
	lime_area,
	navy_area,
	
	//Area of alien building
	building_area,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// area
output reg [11:0] red_area, yellow_area, magenta_area, teal_area, lime_area, navy_area, building_area;

//HSV conversion
output reg[8:0] hsv_h; //0-360
output reg[7:0] hsv_s, hsv_v; //0-255

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// Hue, Saturation and Value thresholds for aliens and building
assign red_detect_h = (hsv_h < 9'd15 || 9'd360 < hsv_h) ? 1'b1 : 1'b0;
assign red_detect_s = (9'd120 < hsv_s) ? 1'b1 : 1'b0;
assign red_detect_v = (9'd80 < hsv_v) ? 1'b1 : 1'b0;

assign yellow_detect_h = (9'd50 < hsv_h  &&  hsv_h < 9'd85) ? 1'b1 : 1'b0;
assign yellow_detect_s = (9'd80 < hsv_s) ? 1'b1 : 1'b0;
assign yellow_detect_v = (9'd200 < hsv_v) ? 1'b1 : 1'b0;

assign magenta_detect_h = (9'd300 < hsv_h  &&  hsv_h < 9'd360) ? 1'b1 : 1'b0;
assign magenta_detect_s = (9'd30 < hsv_s && hsv_s < 9'd180) ? 1'b1 : 1'b0;
assign magenta_detect_v = (9'd80 < hsv_v) ? 1'b1 : 1'b0;

assign teal_detect_h = (9'd160 < hsv_h  &&  hsv_h < 9'd200) ? 1'b1 : 1'b0;
assign teal_detect_s = (9'd100 < hsv_s) ? 1'b1 : 1'b0;
assign teal_detect_v = (9'd20 < hsv_v) ? 1'b1 : 1'b0;

assign lime_detect_h = (9'd90 < hsv_h  &&  hsv_h < 9'd150) ? 1'b1 : 1'b0;
assign lime_detect_s = (9'd100 < hsv_s) ? 1'b1 : 1'b0;
assign lime_detect_v = (9'd80 < hsv_v) ? 1'b1 : 1'b0;

assign navy_detect_h = (9'd210 < hsv_h  &&  hsv_h < 9'd260) ? 1'b1 : 1'b0;
assign navy_detect_s = (9'd100 < hsv_s) ? 1'b1 : 1'b0;
assign navy_detect_v = (9'd30 < hsv_v) ? 1'b1 : 1'b0;

assign building_detect_h = (9'd0 < hsv_h & hsv_h < 9'd360) ? 1'b1 : 1'b0;
assign building_detect_s = (hsv_s < 9'd50) ? 1'b1 : 1'b0;
assign building_detect_v = (9'd150 < hsv_v) ? 1'b1 : 1'b0;

/*assign building_detect_h = (9'd0 < hsv_h & hsv_h < 9'd360) ? 1'b1 : 1'b0;
assign building_detect_s = (hsv_s < 9'd255) ? 1'b1 : 1'b0;
assign building_detect_v = (hsv_v < 9'd50) ? 1'b1 : 1'b0; */


// Detect red areas
wire red_detect, red_detect_hsv;
assign red_detect = {8'hff, 8'h0, 8'h0};
assign red_detect_hsv = (red_detect_h && red_detect_s && red_detect_v) ? 1'b1 : 1'b0;

// Detect yellow areas
wire yellow_detect, yellow_detect_hsv;
assign yellow_detect = {8'hff, 8'hff, 8'h0};
assign yellow_detect_hsv = (yellow_detect_h && yellow_detect_s && yellow_detect_v) ? 1'b1 : 1'b0;

// Detect magenta areas
wire magenta_detect, magenta_detect_hsv;
assign magenta_detect = {8'hff, 8'h0, 8'hff};
assign magenta_detect_hsv = (magenta_detect_h && magenta_detect_s && magenta_detect_v) ? 1'b1 : 1'b0;

// Detect teal areas
wire teal_detect, teal_detect_hsv;
assign teal_detect = {8'h00, 8'h80, 8'h80};
assign teal_detect_hsv = (teal_detect_h && teal_detect_s && teal_detect_v) ? 1'b1 : 1'b0;

// Detect lime areas
wire lime_detect, lime_detect_hsv;
assign lime_detect = {8'h00, 8'hff, 8'h00};
assign lime_detect_hsv = (lime_detect_h && lime_detect_s && lime_detect_v) ? 1'b1 : 1'b0;

// Detect navy areas
wire navy_detect, navy_detect_hsv;
assign navy_detect = {8'h00, 8'h00, 8'h80};
assign navy_detect_hsv = (navy_detect_h && navy_detect_s && navy_detect_v) ? 1'b1 : 1'b0;

// Detect builidng areas
wire building_detect, building_detect_hsv;
assign building_detect = {8'hff, 8'hff, 8'hff};
assign building_detect_hsv = (building_detect_h && building_detect_s && building_detect_v) ? 1'b1 : 1'b0;

// Find boundary of cursor box

// Highlight detected aliens and buildings
wire [23:0] alien_high_hsv;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4

assign alien_high_hsv = red_detect_hsv ? {8'hff, 8'h00, 8'h00} :
								yellow_detect_hsv ? {8'hff, 8'hff, 8'h00} :
								magenta_detect_hsv ? {8'hff, 8'h00, 8'hff} :
								teal_detect_hsv ? {8'h00, 8'h80, 8'h80} :
								lime_detect_hsv ? {8'h00, 8'hff, 8'h00} :
								navy_detect_hsv ? {8'h00, 8'h00, 8'h80} :
								building_detect_hsv ? {8'hff, 8'hff, 8'hff} :
								{grey, grey, grey};


// Show bounding boxes for aliens and buildings
wire [23:0] new_image;
wire redBB_active, yellowBB_active, magentaBB_active, tealBB_active, limeBB_active, navyBB_active, buildingBB_active;
assign redBB_active = (((x == r_left) | (x == r_right)) & (r_bottom >= y & y >= r_top)) | (((y == r_top) | (y == r_bottom)) & (r_left <= x & x <= r_right));
assign yellowBB_active = (((x == y_left) | (x == y_right)) & (y_bottom >= y & y >= y_top)) | (((y == y_top) | (y == y_bottom)) & (y_left <= x & x <= y_right));
assign magentaBB_active = (((x == m_left) | (x == m_right)) & (m_bottom >= y & y >= m_top)) | (((y == m_top) | (y == m_bottom)) & (m_left <= x & x <= m_right));
assign tealBB_active = (((x == t_left) | (x == t_right)) & (t_bottom >= y & y >= t_top)) | (((y == t_top) | (y == t_bottom)) & (t_left <= x & x <= t_right));
assign limeBB_active = (((x == l_left) | (x == l_right)) & (l_bottom >= y & y >= l_top)) | (((y == l_top) | (y == l_bottom)) & (l_left <= x & x <= l_right));
assign navyBB_active = (((x == n_left) | (x == n_right)) & (n_bottom >= y & y >= n_top)) | (((y == n_top) | (y == n_bottom)) & (n_left <= x & x <= n_right));
assign buildingBB_active = (((x == b_left) | (x == b_right)) & (b_bottom >= y & y >= b_top)) | (((y == b_top) | (y == b_bottom)) & (b_left <= x & x <= b_right));

assign new_image = redBB_active ? {8'hff, 8'h00, 8'h00} :
						yellowBB_active ? {8'hff, 8'hff, 8'h00} :
						magentaBB_active ? {8'hff, 8'h00, 8'hff} :
						tealBB_active ? {8'h00, 8'h80, 8'h80} :
						limeBB_active ? {8'h00, 8'hff, 8'h00} :
						navyBB_active ? {8'h00, 8'h00, 8'h80} :
						buildingBB_active ? {8'hff, 8'hff, 8'hff} :
						alien_high_hsv;


// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last pixels (aliens & buildings)
reg [10:0] r_x_min, r_y_min, r_x_max, r_y_max;
reg [10:0] y_x_min, y_y_min, y_x_max, y_y_max;
reg [10:0] m_x_min, m_y_min, m_x_max, m_y_max;
reg [10:0] t_x_min, t_y_min, t_x_max, t_y_max;
reg [10:0] l_x_min, l_y_min, l_x_max, l_y_max;
reg [10:0] n_x_min, n_y_min, n_x_max, n_y_max;
reg [10:0] b_x_min, b_y_min, b_x_max, b_y_max;

always@(posedge clk) begin
	if (red_detect_hsv & in_valid) begin //Update bounds when the pixel is red
		if (x < r_x_min) r_x_min <= x;
		if (x > r_x_max) r_x_max <= x;
		if (y < r_y_min) r_y_min <= y;
		r_y_max <= y;
	end

	else if (yellow_detect_hsv & in_valid) begin //Update bounds when the pixel is yellow
		if (x < y_x_min) y_x_min <= x;
		if (x > y_x_max) y_x_max <= x;
		if (y < y_y_min) y_y_min <= y;
		y_y_max <= y;
	end

	else if (magenta_detect_hsv & in_valid) begin //Update bounds when the pixel is magenta
		if (x < m_x_min) m_x_min <= x;
		if (x > m_x_max) m_x_max <= x;
		if (y < m_y_min) m_y_min <= y;
		m_y_max <= y;
	end

	else if (teal_detect_hsv & in_valid) begin //Update bounds when the pixel is teal
		if (x < t_x_min) t_x_min <= x;
		if (x > t_x_max) t_x_max <= x;
		if (y < t_y_min) t_y_min <= y;
		t_y_max <= y;
	end

	else if (lime_detect_hsv & in_valid) begin //Update bounds when the pixel is lime
		if (x < l_x_min) l_x_min <= x;
		if (x > l_x_max) l_x_max <= x;
		if (y < l_y_min) l_y_min <= y;
		l_y_max <= y;
	end

	else if (navy_detect_hsv & in_valid) begin //Update bounds when the pixel is navy
		if (x < n_x_min) n_x_min <= x;
		if (x > n_x_max) n_x_max <= x;
		if (y < n_y_min) n_y_min <= y;
		n_y_max <= y;
	end

	else if (building_detect_hsv & in_valid) begin //Update bounds when the pixel is white (Building)
		if (x < b_x_min) b_x_min <= x;
		if (x > b_x_max) b_x_max <= x;
		if (y < b_y_min) b_y_min <= y;
		b_y_max <= y;
	end


	if (sop & in_valid) begin //Reset bounds on start of packet
		r_x_min <= IMAGE_W-11'h1;
		r_x_max <= 0;
		r_y_min <= IMAGE_H-11'h1;
		r_y_max <= 0;

		y_x_min <= IMAGE_W-11'h1;
		y_x_max <= 0;
		y_y_min <= IMAGE_H-11'h1;
		y_y_max <= 0;

		m_x_min <= IMAGE_W-11'h1;
		m_x_max <= 0;
		m_y_min <= IMAGE_H-11'h1;
		m_y_max <= 0;

		t_x_min <= IMAGE_W-11'h1;
		t_x_max <= 0;
		t_y_min <= IMAGE_H-11'h1;
		t_y_max <= 0;

		l_x_min <= IMAGE_W-11'h1;
		l_x_max <= 0;
		l_y_min <= IMAGE_H-11'h1;
		l_y_max <= 0;

		n_x_min <= IMAGE_W-11'h1;
		n_x_max <= 0;
		n_y_min <= IMAGE_H-11'h1;
		n_y_max <= 0;

		b_x_min <= IMAGE_W-11'h1;
		b_x_max <= 0;
		b_y_min <= IMAGE_H-11'h1;
		b_y_max <= 0;

	end
end

//Process bounding box at the end of the frame.
reg [2:0] msg_state;
reg [10:0] left, right, top, bottom;
reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		left <= x_min;
		right <= x_max;
		top <= y_min;
		bottom <= y_max;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 3'b01;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 3'b00) msg_state <= msg_state + 3'b01;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

`define RED_BOX_MSG_ID "RBB"

parameter RED = 8'b00000000;
parameter YELLOW = 8'b00000001;
parameter MAGENTA = 8'b00000010;
parameter TEAL = 8'b00000011;
parameter LIME = 8'b00000100;
parameter NAVY = 8'b00000101;
parameter BUILDING = 8'b00000110;

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		3'b000: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
	
		3'b001: begin
			msg_buf_in = {8'b0, RED, red_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b010: begin
			msg_buf_in = {8'b0, YELLOW, yellow_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b011: begin
			msg_buf_in = {8'b0, MAGENTA, magenta_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b100: begin
			msg_buf_in = {8'b0, TEAL, teal_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b101: begin
			msg_buf_in = {8'b0, LIME, lime_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b110: begin
			msg_buf_in = {8'b0, NAVY, navy_area};
			msg_buf_wr = 1'b0;
		end
		
		3'b111: begin
			msg_buf_in = {8'b0, BUILDING, building_area};
			msg_buf_wr = 1'b0;
		end


	endcase
end

//RGB to HSV conversion
HSV hsv_inst(
.clk(clk),
.rst(reset_n),
.rgb_r(red),
.rgb_g(green),
.rgb_b(blue),
.hsv_h(hsv_h),
.hsv_s(hsv_s),
.hsv_v(hsv_v)
);

//AREA needs to be fixed
AREA area_inst(
.clk(clk),
.r_x_min(r_x_min),
.r_x_max(r_x_max),
.r_y_min(r_y_min),
.r_y_max(r_y_max),

.y_x_min(y_x_min),
.y_x_max(y_x_max),
.y_y_min(y_y_min),
.y_y_max(y_y_max),

.m_x_min(m_x_min),
.m_x_max(m_x_max),
.m_y_min(m_y_min),
.m_y_max(m_y_max),

.t_x_min(t_x_min),
.t_x_max(t_x_max),
.t_y_min(t_y_min),
.t_y_max(t_y_min),

.l_x_min(l_x_min),
.l_x_max(l_x_max),
.l_y_min(l_y_min),
.l_y_max(l_y_max),


.n_x_min(n_x_min),
.n_x_max(n_x_max),
.n_y_min(n_y_min),
.n_y_max(n_y_max),

.b_x_min(b_x_min),
.b_x_max(b_x_max),
.b_y_min(b_y_min),
.b_y_max(b_y_max),

.red_area(red_area),
.yellow_area(yellow_area),
.magenta_area(magenta_area),
.teal_area(teal_area),
.lime_area(lime_area),
.navy_area(navy_area),
.building_area(building_area)
);

//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

