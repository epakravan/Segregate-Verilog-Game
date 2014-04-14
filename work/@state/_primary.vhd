library verilog;
use verilog.vl_types.all;
entity State is
    port(
        Clk             : in     vl_logic;
        rand            : in     vl_logic;
        LEFT_Btn        : in     vl_logic;
        RIGHT_Btn       : in     vl_logic;
        Reset           : in     vl_logic;
        Start           : in     vl_logic;
        Ack             : in     vl_logic;
        Pulse           : in     vl_logic;
        score           : out    vl_logic_vector(6 downto 0);
        posL            : out    vl_logic_vector(5 downto 0);
        posR            : out    vl_logic_vector(5 downto 0);
        lightL          : out    vl_logic_vector(63 downto 0);
        lightR          : out    vl_logic_vector(63 downto 0);
        colorL          : out    vl_logic_vector(63 downto 0);
        colorR          : out    vl_logic_vector(63 downto 0);
        temp            : out    vl_logic;
        q_I             : out    vl_logic;
        q_Play          : out    vl_logic;
        q_Left          : out    vl_logic;
        q_Right         : out    vl_logic;
        q_Skip          : out    vl_logic;
        q_Done          : out    vl_logic
    );
end State;
