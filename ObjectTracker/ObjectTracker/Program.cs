using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ObjectTracker
{
	class Program
	{
		static void Main(string[] args)
		{
			using (SecondWindow w = new SecondWindow())
				w.Run();
		}
	}
}
