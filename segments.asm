section .text
	global GetCS, GetDS, GetSS, GetES, GetFS, GetGS
	GetCS:
		mov rax, cs
		ret
	GetDS:
		mov rax, ds
		ret
	GetSS:
		mov rax, ss
		ret
	GetES:
		mov rax, es
		ret
	GetFS:
		mov rax, fs
		ret
	GetGS:
		mov rax, gs
		ret