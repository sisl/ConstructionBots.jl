let
	logfile = "tmp_out.log"
	errfile = "tmp_err.log"
	redirect_to_files(logfile,errfile) do
		limit = 2
		msg = "This message should be printed"
		@log_info(limit,3,msg)
		@log_info(limit,1+2,msg)
		@log_info(limit,limit-1,msg) # nothing should be printed
		@log_info(limit,limit,msg) # nothing should be printed
		@log_info(limit,3,split(msg," ")...)
		@log_info(limit,3)
	end
	@test countlines(logfile) == 4
	rm(logfile)
	rm(errfile)
end
