----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;


-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

entity machine is
  port (
    sysclk : std_logic;
    reset_S : std_logic;
    reset_L : std_logic;
    pixelclock_en : std_logic;
    cpuioclock_en : std_logic;

    vsync : out  STD_LOGIC;
    hsync : out  STD_LOGIC;
    vgared : out  UNSIGNED (3 downto 0);
    vgagreen : out  UNSIGNED (3 downto 0);
    vgablue : out  UNSIGNED (3 downto 0);

    led : out std_logic_vector(15 downto 0);
    sw : in std_logic_vector(15 downto 0);

    -- 7seg 8x displays, each are 7-seg
    -- all cathodes of each display are connected to a common data bus
    -- active low
    sseg_ca : out std_logic_vector(7 downto 0);
    -- one anode per display (each display has common anode to all segments)
    -- active high
    sseg_an : out std_logic_vector(7 downto 0)
    );
end machine;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

architecture Behavioral of machine is

  -- NEXYS use only
  -- counter for multiplexing to the 7seg displays
  signal segled_counter : unsigned(17 downto 0) := (others => '0');
  signal seg_led_data : unsigned(31 downto 0);



begin

-- ######## ########
-- ######## ########

  -- map the SW-in to led-out
  process(sw) is
  begin
    for i in 0 to 15 loop
      led(i) <= sw(i);
    end loop;
  end process;

  -- circuit to multiplex to the 7-seg displays
  -- only used on the NEXYS board
  --
  -- counter, forever loops
  process(sysclk, cpuioclock_en) is --pixelclock,ioclock)
  begin
    if (rising_edge(sysclk) and cpuioclock_en='1') then --ioclock) then
      segled_counter <= segled_counter + 1;
    end if;
  end process;
  --
  process(sysclk, cpuioclock_en) is
    variable digit : std_logic_vector(3 downto 0);
  begin
    if (rising_edge(sysclk) and cpuioclock_en='1') then --ioclock) then

      if sw(0) = '1' then
        seg_led_data(15 downto 0) <= x"6510";
      elsif ( sw(15) & sw(14) & sw(13) & sw(12) = (segled_counter(17 downto 14)) ) then
        seg_led_data(15 downto 0) <= segled_counter(15 downto 0);
      else
        seg_led_data(15 downto 0) <= x"4502";
      end if;

      if ( sw(1) & sw(2) & sw(3) = (segled_counter(17 downto 15)) ) then
        seg_led_data(31 downto 24) <= segled_counter(14 downto 7);
        seg_led_data(23 downto 16) <= segled_counter(15 downto 8);
      else
        seg_led_data(31 downto 24) <= x"19";
        seg_led_data(23 downto 16) <= x"75";
      end if;        

      -- based on counter value, select just one of the common-annode displays
      case segled_counter(17 downto 15) is
        when "000" => sseg_an <= "11111110";
        when "001" => sseg_an <= "11111101";
        when "010" => sseg_an <= "11111011";
        when "011" => sseg_an <= "11110111";
        when "100" => sseg_an <= "11101111";
        when "101" => sseg_an <= "11011111";
        when "110" => sseg_an <= "10111111";
--      when "111" => sseg_an <= "01111111";
        when others =>sseg_an <= "01111111"; -- as when "111"
      end case;
          
     case segled_counter(17 downto 15) is
        when "000" => digit := std_logic_vector(seg_led_data( 3 downto  0));
        when "001" => digit := std_logic_vector(seg_led_data( 7 downto  4));
        when "010" => digit := std_logic_vector(seg_led_data(11 downto  8));
        when "011" => digit := std_logic_vector(seg_led_data(15 downto 12));
        when "100" => digit := std_logic_vector(seg_led_data(19 downto 16));
        when "101" => digit := std_logic_vector(seg_led_data(23 downto 20));
        when "110" => digit := std_logic_vector(seg_led_data(27 downto 24));
--      when "111" => digit := std_logic_vector(seg_led_data(31 downto 28));
        when others =>digit := std_logic_vector(seg_led_data(31 downto 28)); -- as when "111"
      end case;
      
      -- segments are:
      -- 7 - decimal point
      -- 6 - middle
      -- 5 - upper left
      -- 4 - lower left
      -- 3 - bottom
      -- 2 - lower right
      -- 1 - upper right
      -- 0 - top
      case digit is
        when x"0" => sseg_ca   <= "01000000"; -- MSB should be '1' as it is the dot
        when x"1" => sseg_ca   <= "11111001";
        when x"2" => sseg_ca   <= "10100100";
        when x"3" => sseg_ca   <= "10110000";
        when x"4" => sseg_ca   <= "10011001";
        when x"5" => sseg_ca   <= "10010010";
        when x"6" => sseg_ca   <= "10000010";
        when x"7" => sseg_ca   <= "11111000";
        when x"8" => sseg_ca   <= "10000000";
        when x"9" => sseg_ca   <= "10010000";
        when x"A" => sseg_ca   <= "10001000";
        when x"B" => sseg_ca   <= "10000011";
        when x"C" => sseg_ca   <= "11000110";
        when x"D" => sseg_ca   <= "10100001";
        when x"E" => sseg_ca   <= "10000110";
--      when x"F" => sseg_ca   <= "10001110";
        when others => sseg_ca <= "10001110"; -- as when x"F"
      end case; 
      
    end if;
    
  end process;
  
  viciv0: entity work.bensvic2
    port map (
	    sysclk        => sysclk,
		  reset_S       => reset_S,
		  reset_L       => reset_L,
		  pixelclock_en => pixelclock_en,
		  cpuioclock_en => cpuioclock_en,
      vsync         => vsync,
      hsync         => hsync,
      vgared        => vgared,
      vgagreen      => vgagreen,
      vgablue       => vgablue 
      );

end Behavioral;

