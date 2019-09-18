
using System;
using System.Collections;
using System.Collections.Generic;

namespace PMKS
{
    /// <summary>
    /// 
    /// </summary>
    /// <seealso cref="double" />
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

        /// <summary>
        /// Gets the number of elements contained in the <see cref="T:System.Collections.Generic.ICollection`1" />.
        /// </summary>
        public int Count
        {
            get { return LastIndex + 1; }
        }

        /// <summary>
        /// Gets the times.
        /// </summary>
        /// <value>
        /// The times.
        /// </value>
        public List<double> Times
        {
            get { return timeKeys; }
        }

        /// <summary>
        /// Gets the parameters.
        /// </summary>
        /// <value>
        /// The parameters.
        /// </value>
        public List<double[,]> Parameters
        {
            get { return parameterValues; }
        }
        public List<double[,]> ICLoc = new List<double[,]>();
        public List<double[,]> SecICLoc = new List<double[,]>();
        public List<string[,]> SecICname = new List<string[,]>();

       
        public List<double[]> ICVel = new List<double[]>();

          


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
                } while (i > 0 && Times[i] >= time);
                Times.Insert(i + 1, time);
                Parameters.Insert(i + 1, parameters);
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
                } while (Times[i] <= time);
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            LastIndex++;
        }

        /// <summary>
        /// Adds an item to the <see cref="T:System.Collections.Generic.ICollection`1" />.
        /// </summary>
        /// <param name="item">The object to add to the <see cref="T:System.Collections.Generic.ICollection`1" />.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        public void Add(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Removes all items from the <see cref="T:System.Collections.Generic.ICollection`1" />.
        /// </summary>
        /// <exception cref="System.NotImplementedException"></exception>
        public void Clear()
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Determines whether the <see cref="T:System.Collections.Generic.ICollection`1" /> contains a specific value.
        /// </summary>
        /// <param name="item">The object to locate in the <see cref="T:System.Collections.Generic.ICollection`1" />.</param>
        /// <returns>
        /// true if <paramref name="item" /> is found in the <see cref="T:System.Collections.Generic.ICollection`1" />; otherwise, false.
        /// </returns>
        /// <exception cref="System.NotImplementedException"></exception>
        public bool Contains(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Copies the elements of the <see cref="T:System.Collections.Generic.ICollection`1" /> to an <see cref="T:System.Array" />, starting at a particular <see cref="T:System.Array" /> index.
        /// </summary>
        /// <param name="array">The one-dimensional <see cref="T:System.Array" /> that is the destination of the elements copied from <see cref="T:System.Collections.Generic.ICollection`1" />. The <see cref="T:System.Array" /> must have zero-based indexing.</param>
        /// <param name="arrayIndex">The zero-based index in <paramref name="array" /> at which copying begins.</param>
        /// <exception cref="System.NotImplementedException"></exception>
        public void CopyTo(KeyValuePair<double, double[,]>[] array, int arrayIndex)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Gets a value indicating whether the <see cref="T:System.Collections.Generic.ICollection`1" /> is read-only.
        /// </summary>
        /// <exception cref="System.NotImplementedException"></exception>
        public bool IsReadOnly
        {
            get { throw new NotImplementedException(); }
        }

        /// <summary>
        /// Removes the first occurrence of a specific object from the <see cref="T:System.Collections.Generic.ICollection`1" />.
        /// </summary>
        /// <param name="item">The object to remove from the <see cref="T:System.Collections.Generic.ICollection`1" />.</param>
        /// <returns>
        /// true if <paramref name="item" /> was successfully removed from the <see cref="T:System.Collections.Generic.ICollection`1" />; otherwise, false. This method also returns false if <paramref name="item" /> is not found in the original <see cref="T:System.Collections.Generic.ICollection`1" />.
        /// </returns>
        /// <exception cref="System.NotImplementedException"></exception>
        public bool Remove(KeyValuePair<double, double[,]> item)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Returns an enumerator that iterates through the collection.
        /// </summary>
        /// <returns>
        /// An enumerator that can be used to iterate through the collection.
        /// </returns>
        public IEnumerator<KeyValuePair<double, double[,]>> GetEnumerator()
        {
            return new TimeKeyValueEnumerator(Times.ToArray(), Parameters.ToArray());
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        internal double[,] this[double t]
        {
            get { return Parameters[Times.IndexOf(t)]; }
        }

        /// <summary>
        /// Determines the index of a specific item in the <see cref="T:System.Collections.Generic.IList`1" />.
        /// </summary>
        /// <param name="item">The object to locate in the <see cref="T:System.Collections.Generic.IList`1" />.</param>
        /// <returns>
        /// The index of <paramref name="item" /> if found in the list; otherwise, -1.
        /// </returns>
        public int IndexOf(KeyValuePair<double, double[,]> item)
        {
            var index = Times.IndexOf(item.Key);
            if (index == -1) return -1;
            if (item.Value != Parameters[index])
                return -1;
            return index;
        }

        /// <summary>
        /// Inserts an item to the <see cref="T:System.Collections.Generic.IList`1" /> at the specified index.
        /// </summary>
        /// <param name="index">The zero-based index at which <paramref name="item" /> should be inserted.</param>
        /// <param name="item">The object to insert into the <see cref="T:System.Collections.Generic.IList`1" />.</param>
        public void Insert(int index, KeyValuePair<double, double[,]> item)
        {
            Times.Insert(index, item.Key);
            Parameters.Insert(index, item.Value);
        }

        /// <summary>
        /// Removes the <see cref="T:System.Collections.Generic.IList`1" /> item at the specified index.
        /// </summary>
        /// <param name="index">The zero-based index of the item to remove.</param>
        public void RemoveAt(int index)
        {
            Times.RemoveAt(index);
            Parameters.RemoveAt(index);
            LastIndex--;
        }

        /// <summary>
        /// Gets or sets the <see cref="double"/> at the specified index.
        /// </summary>
        /// <value>
        /// The <see cref="double"/>.
        /// </value>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        /// <exception cref="System.InvalidOperationException"></exception>
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
        internal TimeKeyValueEnumerator(double[] timeKeys, double[][,] parameterValues)
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
            get { return new KeyValuePair<double, double[,]>(timeKeys[position], parameterValues[position]); }
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
