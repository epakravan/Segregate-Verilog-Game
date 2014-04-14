library verilog;
use verilog.vl_types.all;
entity clock is
    generic(
        WIDTH           : integer := 30
    );
    port(
        Clk             : in     vl_logic;
        Reset           : in     vl_logic;
        Pulse           : out    vl_logic;
        N               : in     vl_logic_vector
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of WIDTH : constant is 1;
end clock;
