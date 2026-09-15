-- Clean shutdown: while the file named by SOLB_STOP exists, MAME leaves on its
-- own and flushes whatever the disk images still owe. Killing the process in
-- the middle of a write is what loses an installation.
local flag = os.getenv("SOLB_STOP") or ""
if flag == "" then return end

while true do
	local f = io.open(flag, "r")
	if f then
		f:close()
		manager.machine:exit()
		return
	end
	emu.wait(0.5)
end
