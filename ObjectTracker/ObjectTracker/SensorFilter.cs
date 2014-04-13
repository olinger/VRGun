using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ObjectTracker
{
	public class SensorFilter
	{
		private float[] circBuff;
		private int index;
		private int count;

		public SensorFilter(int cap)
		{
			circBuff = new float[cap];
			index = 0;
		}

		public void Add(float elem)
		{
			circBuff[index++] = elem;
			if (index >= circBuff.Length)
				index = 0;

			if (count < circBuff.Length)
				count++;
		}

		public float Mean
		{
			get
			{
				float sum = 0f;

				for (int i = 0; i < count; i++)
					sum += circBuff[i];

				return sum / (float)count;
			}
		}
	}
}
