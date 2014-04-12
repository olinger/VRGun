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
			using (MainWindow w = new MainWindow())
				w.Run();
		}
	}
}
