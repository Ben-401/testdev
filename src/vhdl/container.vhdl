----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

entity container is
  Port ( 
        clk_in : STD_LOGIC;         
         btnCpuReset_in : in  STD_LOGIC; -- normally high, active low
--         irq : in  STD_LOGIC;
--         nmi : in  STD_LOGIC;
         ja1_out : out std_logic;
         ja2_out : out std_logic;
         ja3_out : out std_logic;
         ja4_out : out std_logic;

         ja7_out : out std_logic;
         ja8_out : out std_logic;
         ja9_out : out std_logic;
         jaa_out : out std_logic;
			
         ----------------------------------------------------------------------
         -- VGA output
         ----------------------------------------------------------------------
-- container
         vsync : out  STD_LOGIC;
         hsync : out  STD_LOGIC;
         vgared : out  UNSIGNED (3 downto 0);
         vgagreen : out  UNSIGNED (3 downto 0);
         vgablue : out  UNSIGNED (3 downto 0);

   ----------------------------------------------------------------------
   -- MISC input/output
         led_out : out std_logic_vector(15 downto 0);
	 

         sw_in0  : in std_logic;
         sw_in1  : in std_logic;
         sw_in2  : in std_logic;
         sw_in3  : in std_logic;
         sw_in4  : in std_logic;
         sw_in5  : in std_logic;
         sw_in6  : in std_logic;
         sw_in7  : in std_logic;
         sw_in8  : in std_logic;
         sw_in9  : in std_logic;
         sw_inA  : in std_logic;
         sw_inB  : in std_logic;
         sw_inC  : in std_logic;
         sw_inD  : in std_logic;
         sw_inE  : in std_logic;
         sw_inF  : in std_logic;
			
--         btn_in : in std_logic_vector(4 downto 0);
			
-- container

--         UART_TXD : out std_logic;
--         RsRx : in std_logic;
         
         sseg_ca : out std_logic_vector(7 downto 0);
         sseg_an : out std_logic_vector(7 downto 0)
         );
end container;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

architecture Behavioral of container is

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  component clkgen is
    Port (
      clk_in : STD_LOGIC;         
      reset_ext_in : in  STD_LOGIC;
         
      clk0_out : out std_logic;
      clk1_out : out std_logic;
      locked_out : out std_logic;
      reset_out_A : out std_logic; -- short-press
      reset_out_B : out std_logic  -- long-press
    );
  end component;


 



  


-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  component machine is
  Port (

--         pixelclock : STD_LOGIC;
--         pixelclock2x : STD_LOGIC;
--         cpuclock : std_logic;
--         clock50mhz : std_logic;
--         ioclock : std_logic;
--         uartclock : std_logic;
--         btnCpuReset : in  STD_LOGIC;

    sysclk : in std_logic;
    reset_S : std_logic;
    reset_L : std_logic;
    pixelclock_en : in std_logic;
    cpuioclock_en : in std_logic;

-- machine
         irq : in  STD_LOGIC;
         nmi : in  STD_LOGIC;

--         no_kickstart : in std_logic;
--
--         ddr_counter : in unsigned(7 downto 0);
--         ddr_state : in unsigned(7 downto 0);
         
-- machine
         ----------------------------------------------------------------------
         -- VGA output
         ----------------------------------------------------------------------
         vsync : out  STD_LOGIC;
         hsync : out  STD_LOGIC;
         vgared : out  UNSIGNED (3 downto 0);
         vgagreen : out  UNSIGNED (3 downto 0);
         vgablue : out  UNSIGNED (3 downto 0);

--         ---------------------------------------------------------------------------
--         -- IO lines to the ethernet controller
--         ---------------------------------------------------------------------------
--         eth_mdio : inout std_logic;
--         eth_mdc : out std_logic;
--         eth_reset : out std_logic;
--         eth_rxd : in unsigned(1 downto 0);
--         eth_txd : out unsigned(1 downto 0);
--         eth_txen : out std_logic;
--         eth_rxer : in std_logic;
--         eth_rxdv : in std_logic;
--         eth_interrupt : in std_logic;
--
-- BG removed because does not provide core functionality ethernet/framepacker
         
         -------------------------------------------------------------------------
         -- Lines for the SDcard interface itself
         -------------------------------------------------------------------------
-- machine
--         cs_bo : out std_logic;
--         sclk_o : out std_logic;
--         mosi_o : out std_logic;
--         miso_i : in  std_logic;

         ---------------------------------------------------------------------------
         -- Lines for other devices that we handle here
         ---------------------------------------------------------------------------
--         aclMISO : in std_logic;
--         aclMOSI : out std_logic;
--         aclSS : out std_logic;
--         aclSCK : out std_logic;
--         aclInt1 : in std_logic;
--         aclInt2 : in std_logic;
    
-- machine
--         micData : in std_logic;
--         micClk : out std_logic;
--         micLRSel : out std_logic;
--
--         ampPWM : out std_logic;
--         ampSD : out std_logic;
--
--         tmpSDA : out std_logic;
--         tmpSCL : out std_logic;
--         tmpInt : in std_logic;
--         tmpCT : in std_logic;

         ----------------------------------------------------------------------
         -- Flash RAM for holding config
         ----------------------------------------------------------------------
-- machine
--         QspiSCK : out std_logic;
--         QspiDB : inout std_logic_vector(3 downto 0);
--         QspiCSn : out std_logic;
--
-- BG removed as ???

--         -- Temperature of FPGA
--         fpga_temperature : in std_logic_vector(11 downto 0);
         
         ---------------------------------------------------------------------------
         -- Interface to Slow RAM (wrapper around a 128MB DDR2 RAM chip)
         ---------------------------------------------------------------------------
-- machine
--         slowram_addr : out std_logic_vector(26 downto 0);
--         slowram_we : out std_logic;
--         slowram_request_toggle : out std_logic;
--         slowram_done_toggle : in std_logic;
--         slowram_datain : out std_logic_vector(7 downto 0);
--         slowram_addr_reflect : in std_logic_vector(26 downto 0);
--         slowram_datain_reflect : in std_logic_vector(7 downto 0);
--
--         -- simple-dual-port cache RAM interface so that CPU doesn't have to read
--         -- data cross-clock
--         cache_address        : out std_logic_vector(8 downto 0);
--         cache_read_data      : in std_logic_vector(150 downto 0);   
         
         ----------------------------------------------------------------------
         -- PS/2 adapted USB keyboard & joystick connector.
         -- For now we will use a keyrah adapter to connect to the keyboard.
         ----------------------------------------------------------------------
-- machine
--         ps2data : in std_logic;
--         ps2clock : in std_logic;         

         ----------------------------------------------------------------------
         -- PMOD interface for keyboard, joystick, expansion port etc board.
         ----------------------------------------------------------------------
--         pmod_clock : in std_logic;
--         pmod_start_of_sequence : in std_logic;
--         pmod_data_in : in std_logic_vector(3 downto 0);
--         pmod_data_out : out std_logic_vector(1 downto 0);
--			
--         pmoda : inout std_logic_vector(7 downto 0);
--         uart_rx : in std_logic;
--         uart_tx : out std_logic;
--
-- BG removed this as it seems not required for core functionality

         ----------------------------------------------------------------------
         -- Debug interfaces on Nexys4 board
         ----------------------------------------------------------------------
         led : out std_logic_vector(15 downto 0);
         sw : in std_logic_vector(15 downto 0);
	 
--         btn : in std_logic_vector(4 downto 0);
--
--         UART_TXD : out std_logic;
--         RsRx : in std_logic;
         
         sseg_ca : out std_logic_vector(7 downto 0);
         sseg_an : out std_logic_vector(7 downto 0)
         );
  end component; -- machine

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####
-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  signal sw_int :    std_logic_vector(15 downto 0);
--  signal led_int :    std_logic_vector(15 downto 0);

  -- buffered external OSC 100mhz
  signal clk100buf : std_logic;




  -- toplevel reset signals (clk100buf domain)
  signal reset_int_A : std_logic;
  signal reset_int_B : std_logic;
  signal mrst_s_common_async : std_logic;
  signal mrst_l_common_async : std_logic;


  -- reset synchroniser (for clk1 domain)
  signal CLK1mrst_s : std_logic_vector(1 downto 0);
  signal CLK1mrst_l : std_logic_vector(1 downto 0);
  signal CLK1mrst_s_out : std_logic;
  signal CLK1mrst_l_out : std_logic;


  -- internal locked signal from MMCM
  signal locked_int : std_logic;

  -- buffered internal CLK from MMCM-CLK1
  signal CLK1int : std_logic; -- primary clock is 148.214mhz
  signal CLK1divcnt : unsigned(1 downto 0); -- counter for clock-divisor(s)
  signal CLK1div3_en : std_logic;

  -- signals for "CLK1 domain" only
  -- debounce counter for external signals
  -- initialise to 4 so that after 4x CLK1's, the inputs are sampled
  -- this is required as: 2x clocks for meta FFs, plus 2x spares
  -- also assumes reset is not released until ATLEAST 4x CLK1s after config
  -- which allows external inputs to be fed into FFs before reset release
  signal CLK1sample_counter : unsigned(7 downto 0) := "00000100";

  -- switch inputs
  signal CLK1sw_meta1 : std_logic_vector(15 downto 0);
  signal CLK1sw_meta0 : std_logic_vector(15 downto 0);
  signal CLK1sw_sample : std_logic_vector(15 downto 0);
  
  -- DEBUG signals
  signal CLK1dbg_ff1 : std_logic;

begin

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  -- input global clock buffer on the external 100M osc
  clkin_buf : IBUFG
  port map
   (I   => clk_in,
    O   => clk100buf
    );

  -- insert our clock/reset handling component
  clkgen0: component clkgen
  port map (
    clk_in   => clk100buf,
    reset_ext_in => btnCpuReset_in,
    clk0_out => open,
    clk1_out => clk1int,
    locked_out => locked_int, -- see below for description
    reset_out_A => reset_int_A,
    reset_out_B => reset_int_B
  );
  
  -- combine the three reset sources
  -- all are ACTIVE-HIGH and should be considered as asynchronous
  mrst_s_common_async <= (not locked_int) or reset_int_A or reset_int_B;
  mrst_l_common_async <= (not locked_int) or reset_int_B;

  -- ######
  -- ## CLK1 domain
  -- ######
  
  -- sync resets to clk1 clock domain
  -- using async assert, and sync deassert
  --
  -- short-press
  process(CLK1int, mrst_s_common_async) is
  begin
    if (mrst_s_common_async = '1') then
      CLK1mrst_s <= (others => '1'); -- assert reset
    else
      if rising_edge(CLK1int) then
        CLK1mrst_s(1) <= '0'; -- deassert
        CLK1mrst_s(0) <= CLK1mrst_s(1);
      end if;
    end if;
  end process;
  --
  -- long-press
  process(CLK1int, mrst_l_common_async) is
  begin
    if (mrst_l_common_async = '1') then
      CLK1mrst_l <= (others => '1'); -- assert reset
    else
      if rising_edge(CLK1int) then
        CLK1mrst_l(1) <= '0'; -- deassert
        CLK1mrst_l(0) <= CLK1mrst_l(1);
      end if;
    end if;
  end process;
  --
  -- ensure both 's' and 'l' resets are released synchronously on the same edge
  process(CLK1int, CLK1mrst_s, CLK1mrst_l) is
  begin
    if rising_edge(CLK1int) then
      CLK1mrst_s_out <= CLK1mrst_l(0) or CLK1mrst_s(0); -- 's' is asserted while 'l' is asserted
      CLK1mrst_l_out <= CLK1mrst_l(0);
    end if;
  end process;

  -- combine individual switch inputs to a common bus (to ease coding)
  -- NOTE that we cannot declare (at the top-level) the 16x switches as a common-bus,
  --      because the iostandards are different for some switches,
  --      and because of this difference, ISE complains with a warning that
  --      different iostandards are used on the same bus.
  sw_int <= sw_inF & sw_inE & sw_inD & sw_inC &
            sw_inB & sw_inA & sw_in9 & sw_in8 &
            sw_in7 & sw_in6 & sw_in5 & sw_in4 &
            sw_in3 & sw_in2 & sw_in1 & sw_in0;

  -- metastability for the external inputs
  -- use shift registers, no reset clause
  process (CLK1int) is
  begin
    if rising_edge(CLK1int) then
      -- the "SW"itch bus (15..0)
      CLK1sw_meta1 <= sw_int;
      CLK1sw_meta0 <= CLK1sw_meta1;
        -- insert buttons here below
    end if;
  end process;
  
  -- counter to minimise debouncing effects on the inputs
  -- basically a down-counter, forever loops
  process(CLK1int, CLK1sample_counter) is
  begin
    if rising_edge(CLK1int) then
      CLK1sample_counter <= CLK1sample_counter - 1;
    end if;
  end process;

  -- sample the metastable inputs whenever the sample_counter is '0'
  -- use shift registers, no reset clause
  process(CLK1int) is
  begin
    if rising_edge(CLK1int) then
      if CLK1sample_counter = "00000000" then
        -- sample bouncing switch input signals (POST METASTABLE)
        CLK1sw_sample <= CLK1sw_meta0;
        -- insert buttons here below
      end if;
    end if;
  end process;

  -- generate a counter that can be used to derive the slower clocks
  -- this one is used to generate the "/3"
  process(CLK1int, CLK1mrst_l_out) is
  begin
    if CLK1mrst_l_out = '1' then -- reset clause, wait for MMCM to lock
      CLK1divcnt <= "00";
    elsif rising_edge(CLK1int) then
      if CLK1divcnt >= "10" then
        CLK1divcnt <= "00";
        CLK1div3_en <= '1';
      else
        CLK1divcnt <= CLK1divcnt + "01";
        CLK1div3_en <= '0';
      end if;
    end if;
  end process;

  -- DEBUG: generate a TFF from clk1div3_en
  process(CLK1int, CLK1mrst_l_out) is
  begin
    if CLK1mrst_l_out = '1' then -- reset clause
      CLK1dbg_ff1 <= '0';
    elsif rising_edge(CLK1int) then
      if (CLK1div3_en = '1') then
        CLK1dbg_ff1 <= not CLK1dbg_ff1;
      end if;
    end if;
  end process;
  
  ja1_out <= btnCpuReset_in;
  ja2_out <= reset_int_A;
  ja3_out <= reset_int_B;
  ja4_out <= locked_int;
  
  ja7_out <= '1';
  ja8_out <= CLK1mrst_s_out;
  ja9_out <= CLK1mrst_l_out;
  jaa_out <= CLK1dbg_ff1;
  
  
-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####
  
  machine0: machine
    port map (
      sysclk => CLK1int,
      reset_S => CLK1mrst_s_out, -- normally low, reset=1
      reset_L => CLK1mrst_l_out, -- normally low, reset=1
      pixelclock_en => '1',
      cpuioclock_en => CLK1div3_en,
		
--      pixelclock2x    => open,
--      pixelclock      => open,
--      cpuclock        => open,
----      clock50mhz      => clock50mhz,
----      ioclock         => ioclock, -- 32MHz
----      uartclock       => ioclock, -- must be 32MHz
----      uartclock       => cpuclock, -- Match CPU clock (48MHz)
--      ioclock         => open, -- Match CPU clock
--		
--      btncpureset => locked_int, -- locked is active high
		
      irq => CLK1sw_sample(0),
      nmi => CLK1sw_sample(1),

--      no_kickstart => '0',
--      ddr_counter => ddr_counter,
--      ddr_state => ddr_state,
      
      vsync           => vsync,
      hsync           => hsync,
      vgared          => vgared,
      vgagreen        => vgagreen,
      vgablue         => vgablue,

-- machine
      ---------------------------------------------------------------------------
      -- IO lines to the ethernet controller
      ---------------------------------------------------------------------------
--      eth_mdio => eth_mdio,
--      eth_mdc => eth_mdc,
--      eth_reset => eth_reset,
--      eth_rxd => eth_rxd,
--      eth_txd => eth_txd,
--      eth_txen => eth_txen,
--      eth_rxer => eth_rxer,
--      eth_rxdv => eth_rxdv,
--      eth_interrupt => eth_interrupt,
--
-- BG removed because does not provide core functionality
      
      -------------------------------------------------------------------------
      -- Lines for the SDcard interface itself
      -------------------------------------------------------------------------
-- machine
--      cs_bo => sdReset,
--      sclk_o => sdClock,
--      mosi_o => sdMOSI,
--      miso_i => sdMISO,
--
--      aclMISO => aclMISO,
--      aclMOSI => aclMOSI,
--      aclSS => aclSS,
--      aclSCK => aclSCK,
--      aclInt1 => aclInt1,
--      aclInt2 => aclInt2,
--    
--      micData => micData,
--      micClk => micClk,
--      micLRSel => micLRSel,
--
-- machine
--      ampPWM => ampPWM,
--      ampSD => ampSD,
--    
--      tmpSDA => tmpSDA,
--      tmpSCL => tmpSCL,
--      tmpInt => tmpInt,
--      tmpCT => tmpCT,
--      
--      ps2data =>      ps2data,
--      ps2clock =>     ps2clk,
--
--      pmod_clock => jblo(1),
--      pmod_start_of_sequence => jblo(2),
--      pmod_data_in(1 downto 0) => jblo(4 downto 3),
--      pmod_data_in(3 downto 2) => jbhi(8 downto 7),
--      pmod_data_out => jbhi(10 downto 9),
--
--      pmoda(3 downto 0) => jalo(4 downto 1),
--      pmoda(7 downto 4) => jahi(10 downto 7),
--
--      uart_rx => jclo(1),
--      uart_tx => jclo(2),
--
-- BG removed as it seems not required for core functionality

      
-- machine
--      slowram_we => slowram_we,
--      slowram_request_toggle => slowram_request_toggle,
--      slowram_done_toggle => slowram_done_toggle,
--      slowram_datain => slowram_datain,
--      slowram_addr => slowram_addr,
--      slowram_addr_reflect => slowram_addr_reflect,
--      slowram_datain_reflect => slowram_datain_reflect,
--      cache_read_data => cache_read_data,
--      cache_address => cache_address,

----      QspiSCK => QspiSCK,
--      QspiDB => QspiDB,
--      QspiCSn => QspiCSn,
--
-- BG removed as ???
--      fpga_temperature => fpga_temperature,
-- machine
      led => led_out,
      sw => CLK1sw_sample,
--      btn => btn,
--
--      UART_TXD => UART_TXD,
--      RsRx => RsRx,
--         
      sseg_ca => sseg_ca,
      sseg_an => sseg_an
      );
  

end Behavioral;
