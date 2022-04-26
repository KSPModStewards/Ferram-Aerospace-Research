using System.Collections.Generic;

namespace FerramAerospaceResearch.Utils
{
    public class Metrics
    {
        public Dictionary<string, MetricsElement> data = new();
        const int HysteresisFactor = 20;
        public Metrics() { }
        public void Reset() => data.Clear();
        public void AddMeasurement(string name, double t)
        {
            if (!data.TryGetValue(name, out MetricsElement m))
            {
                m = new MetricsElement();
                data.Add(name, m);
            }
            m.iterations++;
            m.hysteresisTime = (m.hysteresisTime * (HysteresisFactor - 1) + t) / HysteresisFactor;
        }
    }

    public class MetricsElement
    {
        public int iterations = 0;
        public double hysteresisTime = 0;

        public MetricsElement() { }
        public override string ToString() => $"iter: {iterations} TimePerRun: {hysteresisTime:F2} ms";
    }
}
