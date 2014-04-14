library verilog;
use verilog.vl_types.all;
entity segregate_top is
    generic(
        N               : vl_logic_vector(0 to 26) := (Hi1, Hi0, Hi1, Hi1, Hi1, Hi1, Hi1, Hi0, Hi1, Hi0, Hi1, Hi1, Hi1, Hi1, Hi0, Hi0, Hi0, Hi0, Hi1, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0)
    );
    port(
        MemOE           : out    vl_logic;
        MemWR           : out    vl_logic;
        RamCS           : out    vl_logic;
        FlashCS         : out    vl_logic;
        QuadSpiFlashCS  : out    vl_logic;
        ClkPort         : in     vl_logic;
        BtnL            : in     vl_logic;
        BtnU            : in     vl_logic;
        BtnR            : in     vl_logic;
        BtnC            : in     vl_logic;
        Ld7             : out    vl_logic;
        Ld6             : out    vl_logic;
        Ld5             : out    vl_logic;
        Ld4             : out    vl_logic;
        Ld3             : out    vl_logic;
        Ld2             : out    vl_logic;
        Ld1             : out    vl_logic;
        Ld0             : out    vl_logic;
        An3             : out    vl_logic;
        An2             : out    vl_logic;
        An1             : out    vl_logic;
        An0             : out    vl_logic;
        Ca              : out    vl_logic;
        Cb              : out    vl_logic;
        Cc              : out    vl_logic;
        Cd              : out    vl_logic;
        Ce              : out    vl_logic;
        Cf              : out    vl_logic;
        Cg              : out    vl_logic;
        Dp              : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of N : constant is 1;
end segregate_top;
