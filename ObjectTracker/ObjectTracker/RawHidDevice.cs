using System;

using System.Runtime.InteropServices;

namespace ObjectTracker
{
	public static class RawHidDevice
	{
		[DllImport("rawhid.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern int rawhid_open(int max, int vid, int pid, int usage_page, int usage);

		[DllImport("rawhid.dll", CallingConvention = CallingConvention.Cdecl)]
		public static unsafe extern int rawhid_recv(int num, void* buf, int len, int timeout);

		[DllImport("rawhid.dll", CallingConvention = CallingConvention.Cdecl)]
		public static unsafe extern int rawhid_send(int num, void* buf, int len, int timeout);

		[DllImport("rawhid.dll", CallingConvention = CallingConvention.Cdecl)]
		public static extern void rawhid_close(int num);
	}
}
