function get_char()
	term = REPL.Terminals.TTYTerminal("xterm",stdin,stdout,stderr)
	REPL.Terminals.raw!(term,true)
	Base.start_reading(stdin)

    bb = bytesavailable(stdin)
    if bb > 0
        data = read(stdin, bb)
        if data[1] == UInt(3)
            println("Ctrl+C - exiting")
            exit()
        end
        # println("Got $bb bytes: $(string(data))")
		empty_stream = false
		char = Char.(data[end])
    else
		empty_stream = true
		char = nothing
	end
	t = REPL.TerminalMenus.terminal
	REPL.TerminalMenus.disableRawMode(t)
	return char, empty_stream
end
