
using System;
using System.Collections;
using System.Collections.Generic;

namespace PlanarMechanismSimulator
{
    public class TimeSortedList : IList<KeyValuePair<double, double[,]>>
    {
        /// <summary>
        /// Gets the last index.
        /// </summary>
        /// <value>
        /// The last index.
        /// </value>
        public int LastIndex
        {
            get { return _lastIndex; }
            private set { _lastIndex = value; }
        }

        private readonly List<double[,]> parameterValues = new List<double[,]>();
        private readonly List<double> timeKeys = new List<double>();
        private int _lastIndex = -1;

        public int Count
        {
            get { return LastIndex + 1; }
        }

        public List<double> Times
        {
            get { return timeKeys; }
        }

        public List<double[,]> Parameters
        {
            get { return parameterValues; }
        }

        //rkprad: possibly there is a better and simpler way to write the series of add /addnearend /addnearbegin functions

        internal void Add(double time, double[,] parameters)
        {
            if (Count == 0 || time > Times[LastIndex])
            //if count =0; then time is added to the first spot
            //if count =/0 then time is added to the next spot based on lastspot value
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else //inserting time at some intermediate value
            {
                int ub = LastIndex; //ub = upperbound say 5
                int lb = 0; //lb = lower bound 
                int i; //counter
                do
                {
                    i = (ub - lb) / 2; //2.5 -> 3
                    if (Times[i] > time) //Times[3]>
                        ub = i;
                    else lb = i;

                } while (ub - lb > 1);
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            LastIndex++;
        }

        internal void AddNearEnd(double time, double[,] parameters)
        {
            if (Count == 0 || time > Times[LastIndex])
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else if (time < Times[0])
            {
                Times.Insert(0, time);
                Parameters.Insert(0, parameters);
            }
            else
            {
                var i = LastIndex;
                do
                {
                    if (Times[i] == time) return;  //if it already exists, just don't add it at all!
                    i--;
                } while (i > 0 && Times[i] > time);
                Times.Insert(i + 1, time);
                Parameters.Insert(i+1, parameters);
            }
            LastIndex++;
        }

        internal void AddNearBegin(double time, double[,] parameters)
        {
            if (Count == 0 || time > Times[LastIndex])
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else if (time < Times[0])
            {
                Times.Insert(0, time);
                Parameters.Insert(0, parameters);
            }
            else
            {
                int i = 0;
                do
                {
                    if (Times[i] == time) return;  //if it already exists, just don't add it at all!
                    i++;
                } while (Times[i] < time);
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
         

                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            LastIndex++;
        }

        public void Add(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        public void Clear()
        {
            throw new NotImplementedException();
        }

        public bool Contains(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        public void CopyTo(KeyValuePair<double, double[,]>[] array, int arrayIndex)
        {
            throw new NotImplementedException();
        }

        public bool IsReadOnly
        {
            get { throw new NotImplementedException(); }
        }

        public bool Remove(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        public IEnumerator<KeyValuePair<double, double[,]>> GetEnumerator()
        {
            return new TimeKeyValueEnumerator(Times.ToArray(), Parameters.ToArray());
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public double[,] this[double t]
        {
            get { return Parameters[Times.IndexOf(t)]; }
        }

        public int IndexOf(KeyValuePair<double, double[,]> item)
        {
            var index = Times.IndexOf(item.Key);
            if (index == -1) return -1;
            if (item.Value != Parameters[index])
                return -1;
            return index;
        }

        public void Insert(int index, KeyValuePair<double, double[,]> item)
        {
            Times.Insert(index, item.Key);
            Parameters.Insert(index, item.Value);
        }

        public void RemoveAt(int index)
        {
            Times.RemoveAt(index);
            Parameters.RemoveAt(index);
            LastIndex--;
        }

        public KeyValuePair<double, double[,]> this[int index]
        {
            get { return new KeyValuePair<double, double[,]>(Times[index], Parameters[index]); }
            set
            {
                throw new InvalidOperationException();
            }
        }
    }

    class TimeKeyValueEnumerator : IEnumerator<KeyValuePair<double, double[,]>>
    {
        private readonly double[][,] parameterValues;
        private readonly double[] timeKeys;

        // Enumerators are positioned before the first element
        // until the first MoveNext() call.
        int position = -1;
        private readonly int length;
        public TimeKeyValueEnumerator(double[] timeKeys, double[][,] parameterValues)
        {
            this.timeKeys = timeKeys;
            this.parameterValues = parameterValues;
            length = timeKeys.GetLength(0);
        }

        public bool MoveNext()
        {
            position++;
            return (position < length);
        }

        public void Reset()
        {
            position = -1;
        }

        object IEnumerator.Current
        {
            get
            {
                return Current;
            }
        }

        public KeyValuePair<double, double[,]> Current
        {
            get
            {
#if trycatch
                try
                {
#endif
                return new KeyValuePair<double, double[,]>(timeKeys[position], parameterValues[position]);
#if trycatch
                }
                catch (IndexOutOfRangeException)
                {
                    throw new InvalidOperationException();
                }
#endif
            }
        }

        #region Implementation of IDisposable

        /// <summary>
        /// Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
        /// </summary>
        /// <filterpriority>2</filterpriority>
        public void Dispose()
        {
            // throw new NotImplementedException();
        }

        #endregion
    }

}