
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;

library UNISIM;
use UNISIM.VComponents.all;

entity clkgen is
  Port (
    clk_in : STD_LOGIC;           -- from/through the IBUFG
    reset_ext_in : in  STD_LOGIC; -- direct from external, is normally high, active low
         
    clk0_out : out std_logic; -- want this 100mhz
    clk1_out : out std_logic; -- want this about 148mhz for 1920x1080@60
	 
    locked_out : out std_logic;
	 
	 reset_out_A : out std_logic; -- short-press
	 reset_out_B : out std_logic  -- long-press
	 
  );
end clkgen;

architecture Behavioral of clkgen is

  -- The external "CPU_Reset"-input has PULL-UP, so:
  -- normally-open  = "1"
  -- button-pressed = "0" = active-low reset-signal
  -- we need metastability registers on this EXTERNAL input
  signal resetextin_meta2 : std_logic := '1'; -- initially pull-up
  signal resetextin_meta1 : std_logic := '1';
  signal resetextin_meta0 : std_logic := '1';
  
  -- registers to debounce the external input
  -- set "bounce_counter" duration to filter the bounce
  signal bounce_counter  : unsigned(7 downto 0) := (others => '0'); -- 8-bit @ 10ns = 2.56us

  -- counter to detect a long-press of the reset-button
  signal press_counter : unsigned(27 downto 0) := (others => '0'); -- 27-bit @ 10ns = 1.34s

  -- our two reset signals are ACTIVE-HIGH
  signal reset_short_press : std_logic := '1'; -- initially asserted
  signal reset_long_press  : std_logic := '1'; -- initially asserted

  -- MMCM signals
  signal clkfbout_int     : std_logic;
  signal clkout0_intA     : std_logic;
  signal clkout0_intB     : std_logic;
  signal clkout1_intA     : std_logic;
  signal clkout1_intB     : std_logic;
  
  signal locked_int   : std_logic;
--  signal locked_meta1 : std_logic;
--  signal locked_meta0 : std_logic;

begin

  -- metastability logic for the external reset input
  -- continously samples without a reset
  -- also includes a 3rd register for edge-detection
  --
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
      resetextin_meta2 <= reset_ext_in; -- the "CPU_Reset"-button
      resetextin_meta1 <= resetextin_meta2;
      resetextin_meta0 <= resetextin_meta1; -- for edge detection
    end if;
  end process;

  -- two 'reset' signals are implemented:
  --  * a short-press implements a warm-reset, and
  --  * a long--press implements a system-reset.
  --
  -- lets make active-high reset signals (opposed to the active-low CPU_Reset button input)
  -- we will generate a signal which is normally LOW,
  -- for the short-press:
  --   it  becomes HIGH after the CPU_Reset button is pressed  for some period of time (after debouncing),
  --   and becomes LOW  after the CPU_Reset button is released for some period of time (after debouncing).
  -- for the long-press:
  --   it  becomes HIGH if the CPU_Reset button is held for some period of time (about 1 sec),
  --   and becomes LOW  at the same time as the short-press-release
  
  -- debounce logic for the meta-stable reset signal:
  --
  process(clk_in) is   
  begin
    if rising_edge(clk_in) then
      if (resetextin_meta0 /= resetextin_meta1) then -- edge detection
        -- edge detected (button has toggled), so reset counter
        bounce_counter <= (others => '0');
      else
        -- not an edge, so reset-signal is temporarily stable
        if (bounce_counter = "11111111") then
          -- external-reset-input has been stable during count-up, so accept new value
          -- here we invert the polarity of EXTernal reset being active-LOW
          --   to reset_short_press being ACTIVE-HIGH
          reset_short_press <= not resetextin_meta0;
        else
          -- wait until reset-input is stable for the count-up
          bounce_counter <= bounce_counter + 1;
        end if;
      end if;
    end if;
  end process;

  -- detect the long-press
  process(clk_in, reset_short_press) is
  begin
    if rising_edge(clk_in) then
      if (reset_short_press = '0') then -- ie reset NOT asserted
        reset_long_press <= '0';
        press_counter <= (others => '0');
      else
        -- reset_short_press is asserted
        if (press_counter = "111111111111111111111111111") then --press_counter'HIGH) then
          reset_long_press <= '1'; -- ACTIVE-HIGH-reset
        else
          reset_long_press <= '0';
          press_counter <= press_counter + 1;
        end if;
      end if;
    end if;
  end process;


  -- insert a MMCM to:
  -- - create a stable output clock at 100mhz
  -- - create a stable output clock at 148mhz
  mmcm_adv_inst: MMCME2_ADV
    generic map(
      BANDWIDTH            => "OPTIMIZED",
      CLKOUT4_CASCADE      => FALSE,
      COMPENSATION         => "INTERNAL",
      STARTUP_WAIT         => FALSE,
      DIVCLK_DIVIDE        => 1,
      CLKFBOUT_MULT_F      => 10.375, -- is common input multiplier
      CLKFBOUT_PHASE       => 0.0,
      CLKFBOUT_USE_FINE_PS => FALSE,

      CLKOUT0_DIVIDE_F     => 10.375, -- is CLK0 output divisor
      CLKOUT0_PHASE        => 0.0,
      CLKOUT0_DUTY_CYCLE   => 0.5,
      CLKOUT0_USE_FINE_PS  => FALSE,

      CLKOUT1_DIVIDE       => 7, -- is CLK1 output divisor
      CLKOUT1_PHASE        => 0.0,
      CLKOUT1_DUTY_CYCLE   => 0.5,
      CLKOUT1_USE_FINE_PS  => FALSE,

      CLKIN1_PERIOD        => 10.000,
      REF_JITTER1          => 0.100
    )
    port map(
      -- Output clocks
      CLKFBOUT            => clkfbout_int,
      CLKFBOUTB           => open,
      CLKOUT0             => clkout0_intA,
      CLKOUT0B            => open,
      CLKOUT1             => clkout1_intA,
      CLKOUT1B            => open,
      CLKOUT2             => open,
      CLKOUT2B            => open,
      CLKOUT3             => open,
      CLKOUT3B            => open,
      CLKOUT4             => open,
      CLKOUT5             => open,
      CLKOUT6             => open,
      -- Input clock control
      CLKFBIN             => clkfbout_int,
      CLKIN1              => clk_in,
      CLKIN2              => clk_in,
      -- Tied to always select the CLKIN1 input clock
      CLKINSEL            => '1',
      -- Ports for dynamic reconfiguration
      DADDR               => (others => '0'),
      DCLK                => '0',
      DEN                 => '0',
      DI                  => (others => '0'),
      DO                  => open,
      DRDY                => open,
      DWE                 => '0',
      -- Ports for dynamic phase shift
      PSCLK               => '0',
      PSEN                => '0',
      PSINCDEC            => '0',
      PSDONE              => open,
      -- Other control and status signals
      LOCKED              => locked_int, -- MMCM generates active-high locked signal
      CLKINSTOPPED        => open,
      CLKFBSTOPPED        => open,
      PWRDWN              => '0',
      RST                 => reset_long_press -- MMCM uses active-high reset signal
    );
     

  clkout0_buf : BUFG
  port map (
    I   => clkout0_intA,
    O   => clkout0_intB
    );
  
  clkout1_buf : BUFG
  port map (
    I   => clkout1_intA,
    O   => clkout1_intB
    );

  -- component outputs
  clk0_out <= clkout0_intB;
  clk1_out <= clkout1_intB;

  locked_out <= locked_int;
  
  reset_out_A <= reset_short_press;
  reset_out_B <= reset_long_press;
  
end Behavioral;
