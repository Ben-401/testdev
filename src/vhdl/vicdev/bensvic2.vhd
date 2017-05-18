--
-- Written by
--    Paul Gardner-Stephen <hld@c64.org>  2013-2014
--
-- *  This program is free software; you can redistribute it and/or modify
-- *  it under the terms of the GNU Lesser General Public License as
-- *  published by the Free Software Foundation; either version 3 of the
-- *  License, or (at your option) any later version.
-- *
-- *  This program is distributed in the hope that it will be useful,
-- *  but WITHOUT ANY WARRANTY; without even the implied warranty of
-- *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- *  GNU General Public License for more details.
-- *
-- *  You should have received a copy of the GNU Lesser General Public License
-- *  along with this program; if not, write to the Free Software
-- *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
-- *  02111-1307  USA.
----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:11:30 01/02/2014 
-- Design Name: 
-- Module Name:    vga - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
--use ieee.std_logic_arith.all;
--use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use Std.TextIO.all;
use work.debugtools.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity bensvic2 is
  Port (
    sysclk        : in std_logic;
	 reset2        : in std_logic;
	 pixelclock_en : in std_logic;
	 cpuioclock_en : in std_logic;
    

    
    ----------------------------------------------------------------------
    -- VGA output
    ----------------------------------------------------------------------
    vsync : out  STD_LOGIC;
    hsync : out  STD_LOGIC;
    vgared : out  UNSIGNED (3 downto 0);
    vgagreen : out  UNSIGNED (3 downto 0);
    vgablue : out  UNSIGNED (3 downto 0)
	 
	 
    );
end bensvic2;

architecture Behavioral of bensvic2 is

---- 1920x1200 / 193.75
--  constant  h_pulse  :  INTEGER   := 208;   --horiztonal sync pulse width in pixels
--  constant  h_bp     :  INTEGER   := 336;   --horiztonal back porch width in pixels
--  constant  h_pixels :  INTEGER   := 1920;  --horiztonal display width in pixels
--  constant  h_fp     :  INTEGER   := 128;   --horiztonal front porch width in pixels
--  constant  h_pol    :  STD_LOGIC := '0';   --horizontal sync pulse polarity (1 = positive, 0 = negative)
--  constant  v_pulse  :  INTEGER   := 3;     --vertical sync pulse width in rows
--  constant  v_bp     :  INTEGER   := 38;    --vertical back porch width in rows
--  constant  v_pixels :  INTEGER   := 1200;  --vertical display width in rows
--  constant  v_fp     :  INTEGER   := 1;     --vertical front porch width in rows
--  constant  v_pol    :  STD_LOGIC := '1';  --vertical sync pulse polarity (1 = positive, 0 = negative)

-- 1920x1080 / 148.25
  constant  h_pulse  :  INTEGER   := 44;   --horiztonal sync pulse width in pixels
  constant  h_bp     :  INTEGER   := 148;   --horiztonal back porch width in pixels
  constant  h_pixels :  INTEGER   := 1920;  --horiztonal display width in pixels
  constant  h_fp     :  INTEGER   := 88;   --horiztonal front porch width in pixels
  constant  h_pol    :  STD_LOGIC := '0';   --horizontal sync pulse polarity (1 = positive, 0 = negative)
  constant  v_pulse  :  INTEGER   := 5;     --vertical sync pulse width in rows
  constant  v_bp     :  INTEGER   := 36;    --vertical back porch width in rows
  constant  v_pixels :  INTEGER   := 1080;  --vertical display width in rows
  constant  v_fp     :  INTEGER   := 4;     --vertical front porch width in rows
  constant  v_pol    :  STD_LOGIC := '1';  --vertical sync pulse polarity (1 = positive, 0 = negative)

  CONSTANT  h_period  :  INTEGER := h_pulse + h_bp + h_pixels + h_fp;  --total number of pixel clocks in a row
  CONSTANT  v_period  :  INTEGER := v_pulse + v_bp + v_pixels + v_fp;  --total number of rows in column

  signal hc0us : unsigned(11 downto 0);
  signal vc0us : unsigned(10 downto 0);
  signal hc1us : unsigned(11 downto 0);
  signal vc1us : unsigned(10 downto 0);

  signal outtype0 : unsigned(1 downto 0);
  signal outtype1 : unsigned(1 downto 0);

  signal vgar_int : unsigned(3 downto 0);
  signal vgag_int : unsigned(3 downto 0);
  signal vgab_int : unsigned(3 downto 0);
  
  
begin

  process (reset2, sysclk, pixelclock_en)

--    VARIABLE hc : std_logic_vector(16 downto 0);--INTEGER RANGE 0 TO h_period := 0;
--    VARIABLE vc : std_logic_vector(16 downto 0);--INTEGER RANGE 0 TO h_period := 0;

  begin
    if (reset2 = '0') then
      hc0us <= (others => '0');
      vc0us <= (others => '0');
    elsif (rising_edge(sysclk) and pixelclock_en = '1') then
      -- H-counter
      if (hc0us < h_period) then
        hc0us <= hc0us + 1;--d_logic_vector(to_unsigned(1));--"00000000000000001";--std_logic_vector(1,17);
      else
        hc0us <= (others => '0');

        -- V-counter
        if (vc0us < v_period) then
          vc0us <= vc0us + 1;--std_logic_vector(1,17);--1; --"00000000000000001";
        else
          vc0us <= (others => '0');
        end if;
      end if;
    end if;
  end process;
  

--  process (reset2, sysclk, pixelclock_en)
--
--    VARIABLE hc0v : INTEGER RANGE 0 TO h_period := 0;
--    VARIABLE vc0v : INTEGER RANGE 0 TO v_period := 0;
--
--  begin
--
--    if (rising_edge(sysclk) and pixelclock_en = '1') then
--      if (reset2 = '0') then
--		  hc0s <= (others => '0');
--		  vc0s <= (others => '0');
--      else
--		
--  		  if (hc0v < h_period) then
--		    hc0v := hc0v + 1;
--		  else
--		    hc0v := 0;--(others => '0');
--
--  		    if (vc0v < v_period) then
--		      vc0v := vc0v + 1; --"00000000000000001";
--		    else
--		      vc0v := 0;--(others => '0');
--		    end if;
--		  end if;
--
--
--		  
--		end if;
--	 end if;

  process (reset2, sysclk, pixelclock_en)
  begin  
	 
    if (reset2 = '0') then
      hsync <= NOT h_pol;  --deassert horizontal sync
      vsync <= NOT v_pol;  --deassert vertical sync
    elsif (rising_edge(sysclk) and pixelclock_en = '1') then
      --horizontal sync signal
      IF(hc0us < ( h_pixels + h_fp) OR
         hc0us > ( h_pixels + h_fp + h_pulse) )
      THEN
        hsync <= NOT h_pol;    --deassert horiztonal sync pulse
      ELSE
        hsync <= h_pol;        --assert horiztonal sync pulse
      END IF;
      
      --vertical sync signal
      IF(vc0us < ( v_pixels + v_fp) OR
         vc0us > ( v_pixels + v_fp + v_pulse) )
      THEN
        vsync <= NOT v_pol;    --deassert vertical sync pulse
      ELSE
        vsync <= v_pol;        --assert vertical sync pulse
      END IF;
    end if;
  end process;

  process (reset2, sysclk, pixelclock_en)
  begin
    if (reset2 = '0') then
      outtype0 <= "00";
    elsif (rising_edge(sysclk) and pixelclock_en = '1') then
      --set pixel coordinates
      IF (hc0us < h_pixels) then
        if ( (hc0us < 10) or
          ((h_pixels - hc0us) < 10) or
          (vc0us < 10) or
          ((v_pixels - vc0us) < 10) )
        then
          outtype0 <= "10";
        else
          outtype0 <= "01";
        end if;
      else
        outtype0 <= "00";
      end if;

      vgar_int   <= hc0us(6 downto 3);--to_unsigned(hc1v,4);         --set horiztonal pixel coordinate
      vgag_int <= vc0us(6 downto 3);--to_unsigned(vc1v,4);         --set horiztonal pixel coordinate
      vgab_int  <= hc0us(7 downto 4);--to_unsigned(vc1v,4);         --set horiztonal pixel coordinate
      
      
    end if;
  end process;
		  
  process (reset2, sysclk, pixelclock_en)
  begin
    if (reset2 = '0') then
      vgared <= to_unsigned(0,4);
      vgagreen <= to_unsigned(0,4);
      vgablue <= to_unsigned(0,4);
    elsif (rising_edge(sysclk) and pixelclock_en = '1') then
      case outtype0 is
        when "01" =>
          vgared   <= vgar_int;
          vgagreen <= vgag_int;
          vgablue  <= vgab_int;
        when "10" =>
          vgared <= to_unsigned(15,4);
          vgagreen <= to_unsigned(15,4);
          vgablue <= to_unsigned(15,4);
        when others =>
          vgared <= to_unsigned(0,4);
          vgagreen <= to_unsigned(0,4);
          vgablue <= to_unsigned(0,4);
      end case;
    end if;
  end process;
  
  
end Behavioral;

