
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.InteropServices;

namespace PlanarMechanismSimulator
{
    public class TimeSortedList : ICollection<KeyValuePair<double, double[,]>>
    {
        private int lastSpot = -1;
        private readonly List<double[,]> parameterValues = new List<double[,]>();
        private readonly List<double> timeKeys = new List<double>();
        public int Size { get { return lastSpot + 1; } }
        public List<double> Times
        {
            get { return timeKeys; }
        }

        public List<double[,]> Parameters
        {
            get { return parameterValues; }
        }

        internal void Add(double time, double[,] parameters)
        {
            if (Size == 0 || time > Times[lastSpot])
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else
            {
                int ub = lastSpot;
                int lb = 0;
                int i;
                do
                {
                    i = (ub-lb)/2;
                    if (Times[i] > time)
                        ub = i;
                    else lb = i;

                } while (ub-lb>1);
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            lastSpot++;
        }
        internal void AddNearEnd(double time, double[,] parameters)
        {
            if (Size == 0 || time > Times[lastSpot])
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else
            {
                int i = lastSpot;
                while (Times[i] > time) i--;
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            lastSpot++;
        }

        internal void AddNearBegin(double time, double[,] parameters)
        {
            if (Size == 0 || time > Times[lastSpot])
            {
                Times.Add(time);
                Parameters.Add(parameters);
            }
            else
            {
                int i = 0;
                while (Times[i] < time) i++;
                Times.Insert(i, time);
                Parameters.Insert(i, parameters);
            }
            lastSpot++;
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

        public int Count
        {
            get { throw new NotImplementedException(); }
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
            throw new NotImplementedException();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            throw new NotImplementedException();
        }

        public double[,] this[double t]
        {
            get { return Parameters[Times.IndexOf(t)]; }
        }

    }
    // Summary:
    //     Represents a collection of key/value pairs that are sorted by key based on
    //     the associated System.Collections.Generic.IComparer<T> implementation.
    //
    // Type parameters:
    //   TKey:
    //     The type of keys in the collection.
    //
    //   TValue:
    //     The type of values in the collection.
    //public class TimeSortedList<TKey, TValue> : IDictionary<TKey, TValue>, ICollection<KeyValuePair<TKey, TValue>>, IEnumerable<KeyValuePair<TKey, TValue>>, IDictionary, ICollection, IEnumerable
    //{
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that is empty, has the default initial capacity, and uses the default
    //    //     System.Collections.Generic.IComparer<T>.
    //    public TimeSortedList()
    //    {
    //    }

    //    //
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that is empty, has the default initial capacity, and uses the specified
    //    //     System.Collections.Generic.IComparer<T>.
    //    //
    //    // Parameters:
    //    //   comparer:
    //    //     The System.Collections.Generic.IComparer<T> implementation to use when comparing
    //    //     keys.-or-null to use the default System.Collections.Generic.Comparer<T> for
    //    //     the type of the key.
    //    public TimeSortedList(IComparer<TKey> comparer)
    //    {
    //    }

    //    //
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that contains elements copied from the specified System.Collections.Generic.IDictionary<TKey,TValue>,
    //    //     has sufficient capacity to accommodate the number of elements copied, and
    //    //     uses the default System.Collections.Generic.IComparer<T>.
    //    //
    //    // Parameters:
    //    //   dictionary:
    //    //     The System.Collections.Generic.IDictionary<TKey,TValue> whose elements are
    //    //     copied to the new System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     dictionary is null.
    //    //
    //    //   System.ArgumentException:
    //    //     dictionary contains one or more duplicate keys.
    //    public TimeSortedList(IDictionary<TKey, TValue> dictionary){}
    //    //
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that is empty, has the specified initial capacity, and uses the default
    //    //     System.Collections.Generic.IComparer<T>.
    //    //
    //    // Parameters:
    //    //   capacity:
    //    //     The initial number of elements that the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     can contain.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentOutOfRangeException:
    //    //     capacity is less than zero.
    //    public TimeSortedList(int capacity){}
    //    //
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that contains elements copied from the specified System.Collections.Generic.IDictionary<TKey,TValue>,
    //    //     has sufficient capacity to accommodate the number of elements copied, and
    //    //     uses the specified System.Collections.Generic.IComparer<T>.
    //    //
    //    // Parameters:
    //    //   dictionary:
    //    //     The System.Collections.Generic.IDictionary<TKey,TValue> whose elements are
    //    //     copied to the new System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    //   comparer:
    //    //     The System.Collections.Generic.IComparer<T> implementation to use when comparing
    //    //     keys.-or-null to use the default System.Collections.Generic.Comparer<T> for
    //    //     the type of the key.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     dictionary is null.
    //    //
    //    //   System.ArgumentException:
    //    //     dictionary contains one or more duplicate keys.
    //    public TimeSortedList(IDictionary<TKey, TValue> dictionary, IComparer<TKey> comparer){}
    //    //
    //    // Summary:
    //    //     Initializes a new instance of the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     class that is empty, has the specified initial capacity, and uses the specified
    //    //     System.Collections.Generic.IComparer<T>.
    //    //
    //    // Parameters:
    //    //   capacity:
    //    //     The initial number of elements that the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     can contain.
    //    //
    //    //   comparer:
    //    //     The System.Collections.Generic.IComparer<T> implementation to use when comparing
    //    //     keys.-or-null to use the default System.Collections.Generic.Comparer<T> for
    //    //     the type of the key.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentOutOfRangeException:
    //    //     capacity is less than zero.
    //    public TimeSortedList(int capacity, IComparer<TKey> comparer) { }

    //    // Summary:
    //    //     Gets or sets the number of elements that the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     can contain.
    //    //
    //    // Returns:
    //    //     The number of elements that the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     can contain.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentOutOfRangeException:
    //    //     System.Collections.Generic.SortedList<TKey,TValue>.Capacity is set to a value
    //    //     that is less than System.Collections.Generic.SortedList<TKey,TValue>.Count.
    //    //
    //    //   System.OutOfMemoryException:
    //    //     There is not enough memory available on the system.
    //    public int Capacity { get; set; }
    //    //
    //    // Summary:
    //    //     Gets the System.Collections.Generic.IComparer<T> for the sorted list.
    //    //
    //    // Returns:
    //    //     The System.IComparable<T> for the current System.Collections.Generic.SortedList<TKey,TValue>.
    //    public IComparer<TKey> Comparer
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    //
    //    // Summary:
    //    //     Gets the number of key/value pairs contained in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     The number of key/value pairs contained in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    /// <summary>
    //    /// Removes the first occurrence of a specific object from the <see cref="T:System.Collections.Generic.ICollection`1"/>.
    //    /// </summary>
    //    /// <returns>
    //    /// true if <paramref name="item"/> was successfully removed from the <see cref="T:System.Collections.Generic.ICollection`1"/>; otherwise, false. This method also returns false if <paramref name="item"/> is not found in the original <see cref="T:System.Collections.Generic.ICollection`1"/>.
    //    /// </returns>
    //    /// <param name="item">The object to remove from the <see cref="T:System.Collections.Generic.ICollection`1"/>.</param><exception cref="T:System.NotSupportedException">The <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only.</exception>
    //    public bool Remove(KeyValuePair<TKey, TValue> item)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Copies the elements of the <see cref="T:System.Collections.ICollection"/> to an <see cref="T:System.Array"/>, starting at a particular <see cref="T:System.Array"/> index.
    //    /// </summary>
    //    /// <param name="array">The one-dimensional <see cref="T:System.Array"/> that is the destination of the elements copied from <see cref="T:System.Collections.ICollection"/>. The <see cref="T:System.Array"/> must have zero-based indexing. </param><param name="index">The zero-based index in <paramref name="array"/> at which copying begins. </param><exception cref="T:System.ArgumentNullException"><paramref name="array"/> is null. </exception><exception cref="T:System.ArgumentOutOfRangeException"><paramref name="index"/> is less than zero. </exception><exception cref="T:System.ArgumentException"><paramref name="array"/> is multidimensional.-or- The number of elements in the source <see cref="T:System.Collections.ICollection"/> is greater than the available space from <paramref name="index"/> to the end of the destination <paramref name="array"/>.-or- The type of the source <see cref="T:System.Collections.ICollection"/> cannot be cast automatically to the type of the destination <paramref name="array"/><paramref name="."/></exception>
    //    public void CopyTo(Array array, int index)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    public int Count
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Gets an object that can be used to synchronize access to the <see cref="T:System.Collections.ICollection"/>.
    //    /// </summary>
    //    /// <returns>
    //    /// An object that can be used to synchronize access to the <see cref="T:System.Collections.ICollection"/>.
    //    /// </returns>
    //    public object SyncRoot { get; private set; }

    //    /// <summary>
    //    /// Gets a value indicating whether access to the <see cref="T:System.Collections.ICollection"/> is synchronized (thread safe).
    //    /// </summary>
    //    /// <returns>
    //    /// true if access to the <see cref="T:System.Collections.ICollection"/> is synchronized (thread safe); otherwise, false.
    //    /// </returns>
    //    public bool IsSynchronized { get; private set; }

    //    /// <summary>
    //    /// Gets an <see cref="T:System.Collections.ICollection"/> object containing the values in the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.ICollection"/> object containing the values in the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </returns>
    //    ICollection IDictionary.Values
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Gets a value indicating whether the <see cref="T:System.Collections.IDictionary"/> object is read-only.
    //    /// </summary>
    //    /// <returns>
    //    /// true if the <see cref="T:System.Collections.IDictionary"/> object is read-only; otherwise, false.
    //    /// </returns>
    //    bool IDictionary.IsReadOnly
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Gets a value indicating whether the <see cref="T:System.Collections.IDictionary"/> object has a fixed size.
    //    /// </summary>
    //    /// <returns>
    //    /// true if the <see cref="T:System.Collections.IDictionary"/> object has a fixed size; otherwise, false.
    //    /// </returns>
    //    public bool IsFixedSize { get; private set; }

    //    /// <summary>
    //    /// Gets a value indicating whether the <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only.
    //    /// </summary>
    //    /// <returns>
    //    /// true if the <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only; otherwise, false.
    //    /// </returns>
    //    bool ICollection<KeyValuePair<TKey, TValue>>.IsReadOnly
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    //
    //    // Summary:
    //    //     Gets a collection containing the keys in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     A System.Collections.Generic.IList<T> containing the keys in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    public IList<TKey> Keys
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Gets an <see cref="T:System.Collections.ICollection"/> object containing the keys of the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.ICollection"/> object containing the keys of the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </returns>
    //    ICollection IDictionary.Keys
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Gets an <see cref="T:System.Collections.Generic.ICollection`1"/> containing the values in the <see cref="T:System.Collections.Generic.IDictionary`2"/>.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.Generic.ICollection`1"/> containing the values in the object that implements <see cref="T:System.Collections.Generic.IDictionary`2"/>.
    //    /// </returns>
    //    ICollection<TValue> IDictionary<TKey, TValue>.Values
    //    {
    //        get { return Values; }
    //    }

    //    //
    //    // Summary:
    //    //     Gets a collection containing the values in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     A System.Collections.Generic.IList<T> containing the values in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    /// <summary>
    //    /// Gets an <see cref="T:System.Collections.Generic.ICollection`1"/> containing the keys of the <see cref="T:System.Collections.Generic.IDictionary`2"/>.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.Generic.ICollection`1"/> containing the keys of the object that implements <see cref="T:System.Collections.Generic.IDictionary`2"/>.
    //    /// </returns>
    //    ICollection<TKey> IDictionary<TKey, TValue>.Keys
    //    {
    //        get { return Keys; }
    //    }

    //    public IList<TValue> Values
    //    {
    //        get { throw new NotImplementedException(); }
    //    }

    //    // Summary:
    //    //     Gets or sets the value associated with the specified key.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key whose value to get or set.
    //    //
    //    // Returns:
    //    //     The value associated with the specified key. If the specified key is not
    //    //     found, a get operation throws a System.Collections.Generic.KeyNotFoundException
    //    //     and a set operation creates a new element using the specified key.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    //
    //    //   System.Collections.Generic.KeyNotFoundException:
    //    //     The property is retrieved and key does not exist in the collection.
    //    public TValue this[TKey key]
    //    {
    //        get { throw new NotImplementedException(); }
    //        set { throw new NotImplementedException(); }
    //    }

    //    // Summary:
    //    //     Adds an element with the specified key and value into the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key of the element to add.
    //    //
    //    //   value:
    //    //     The value of the element to add. The value can be null for reference types.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    //
    //    //   System.ArgumentException:
    //    //     An element with the same key already exists in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    public void Add(TKey key, TValue value){}
    //    //
    //    // Summary:
    //    //     Removes all elements from the System.Collections.Generic.SortedList<TKey,TValue>.
    //    /// <summary>
    //    /// Adds an item to the <see cref="T:System.Collections.Generic.ICollection`1"/>.
    //    /// </summary>
    //    /// <param name="item">The object to add to the <see cref="T:System.Collections.Generic.ICollection`1"/>.</param><exception cref="T:System.NotSupportedException">The <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only.</exception>
    //    public void Add(KeyValuePair<TKey, TValue> item)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Determines whether the <see cref="T:System.Collections.IDictionary"/> object contains an element with the specified key.
    //    /// </summary>
    //    /// <returns>
    //    /// true if the <see cref="T:System.Collections.IDictionary"/> contains an element with the key; otherwise, false.
    //    /// </returns>
    //    /// <param name="key">The key to locate in the <see cref="T:System.Collections.IDictionary"/> object.</param><exception cref="T:System.ArgumentNullException"><paramref name="key"/> is null. </exception>
    //    public bool Contains(object key)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Adds an element with the provided key and value to the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </summary>
    //    /// <param name="key">The <see cref="T:System.Object"/> to use as the key of the element to add. </param><param name="value">The <see cref="T:System.Object"/> to use as the value of the element to add. </param><exception cref="T:System.ArgumentNullException"><paramref name="key"/> is null. </exception><exception cref="T:System.ArgumentException">An element with the same key already exists in the <see cref="T:System.Collections.IDictionary"/> object. </exception><exception cref="T:System.NotSupportedException">The <see cref="T:System.Collections.IDictionary"/> is read-only.-or- The <see cref="T:System.Collections.IDictionary"/> has a fixed size. </exception>
    //    public void Add(object key, object value)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    public void Clear(){}

    //    /// <summary>
    //    /// Returns an <see cref="T:System.Collections.IDictionaryEnumerator"/> object for the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.IDictionaryEnumerator"/> object for the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </returns>
    //    IDictionaryEnumerator IDictionary.GetEnumerator()
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Removes the element with the specified key from the <see cref="T:System.Collections.IDictionary"/> object.
    //    /// </summary>
    //    /// <param name="key">The key of the element to remove. </param><exception cref="T:System.ArgumentNullException"><paramref name="key"/> is null. </exception><exception cref="T:System.NotSupportedException">The <see cref="T:System.Collections.IDictionary"/> object is read-only.-or- The <see cref="T:System.Collections.IDictionary"/> has a fixed size. </exception>
    //    public void Remove(object key)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Gets or sets the element with the specified key.
    //    /// </summary>
    //    /// <returns>
    //    /// The element with the specified key.
    //    /// </returns>
    //    /// <param name="key">The key of the element to get or set. </param><exception cref="T:System.ArgumentNullException"><paramref name="key"/> is null. </exception><exception cref="T:System.NotSupportedException">The property is set and the <see cref="T:System.Collections.IDictionary"/> object is read-only.-or- The property is set, <paramref name="key"/> does not exist in the collection, and the <see cref="T:System.Collections.IDictionary"/> has a fixed size. </exception>
    //    object IDictionary.this[object key]
    //    {
    //        get { throw new NotImplementedException(); }
    //        set { throw new NotImplementedException(); }
    //    }

    //    /// <summary>
    //    /// Determines whether the <see cref="T:System.Collections.Generic.ICollection`1"/> contains a specific value.
    //    /// </summary>
    //    /// <returns>
    //    /// true if <paramref name="item"/> is found in the <see cref="T:System.Collections.Generic.ICollection`1"/>; otherwise, false.
    //    /// </returns>
    //    /// <param name="item">The object to locate in the <see cref="T:System.Collections.Generic.ICollection`1"/>.</param>
    //    public bool Contains(KeyValuePair<TKey, TValue> item)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    /// <summary>
    //    /// Copies the elements of the <see cref="T:System.Collections.Generic.ICollection`1"/> to an <see cref="T:System.Array"/>, starting at a particular <see cref="T:System.Array"/> index.
    //    /// </summary>
    //    /// <param name="array">The one-dimensional <see cref="T:System.Array"/> that is the destination of the elements copied from <see cref="T:System.Collections.Generic.ICollection`1"/>. The <see cref="T:System.Array"/> must have zero-based indexing.</param><param name="arrayIndex">The zero-based index in <paramref name="array"/> at which copying begins.</param><exception cref="T:System.ArgumentNullException"><paramref name="array"/> is null.</exception><exception cref="T:System.ArgumentOutOfRangeException"><paramref name="arrayIndex"/> is less than 0.</exception><exception cref="T:System.ArgumentException"><paramref name="array"/> is multidimensional.-or-The number of elements in the source <see cref="T:System.Collections.Generic.ICollection`1"/> is greater than the available space from <paramref name="arrayIndex"/> to the end of the destination <paramref name="array"/>.-or-Type <paramref name="T"/> cannot be cast automatically to the type of the destination <paramref name="array"/>.</exception>
    //    public void CopyTo(KeyValuePair<TKey, TValue>[] array, int arrayIndex)
    //    {
    //        throw new NotImplementedException();
    //    }

    //    //
    //    // Summary:
    //    //     Determines whether the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     contains a specific key.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key to locate in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     true if the System.Collections.Generic.SortedList<TKey,TValue> contains an
    //    //     element with the specified key; otherwise, false.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    public bool ContainsKey(TKey key)
    //    {
    //        return false;
    //    }

    //    //
    //    // Summary:
    //    //     Determines whether the System.Collections.Generic.SortedList<TKey,TValue>
    //    //     contains a specific value.
    //    //
    //    // Parameters:
    //    //   value:
    //    //     The value to locate in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //     The value can be null for reference types.
    //    //
    //    // Returns:
    //    //     true if the System.Collections.Generic.SortedList<TKey,TValue> contains an
    //    //     element with the specified value; otherwise, false.
    //    public bool ContainsValue(TValue value)
    //    {
    //        return false;
    //    }

    //    //
    //    // Summary:
    //    //     Returns an enumerator that iterates through the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     An System.Collections.Generic.IEnumerator<T> of type System.Collections.Generic.KeyValuePair<TKey,TValue>
    //    //     for the System.Collections.Generic.SortedList<TKey,TValue>.
    //    public IEnumerator<KeyValuePair<TKey, TValue>> GetEnumerator()
    //    {
    //        return null;
    //    }

    //    //
    //    // Summary:
    //    //     Searches for the specified key and returns the zero-based index within the
    //    //     entire System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key to locate in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Returns:
    //    //     The zero-based index of key within the entire System.Collections.Generic.SortedList<TKey,TValue>,
    //    //     if found; otherwise, -1.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    public int IndexOfKey(TKey key)
    //    {
    //        return 0;
    //    }

    //    //
    //    // Summary:
    //    //     Searches for the specified value and returns the zero-based index of the
    //    //     first occurrence within the entire System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Parameters:
    //    //   value:
    //    //     The value to locate in the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //     The value can be null for reference types.
    //    //
    //    // Returns:
    //    //     The zero-based index of the first occurrence of value within the entire System.Collections.Generic.SortedList<TKey,TValue>,
    //    //     if found; otherwise, -1.
    //    public int IndexOfValue(TValue value)
    //    {
    //        return 0;
    //    }

    //    //
    //    // Summary:
    //    //     Removes the element with the specified key from the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key of the element to remove.
    //    //
    //    // Returns:
    //    //     true if the element is successfully removed; otherwise, false. This method
    //    //     also returns false if key was not found in the original System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    public bool Remove(TKey key)
    //    {
    //        return false;
    //    }

    //    //
    //    // Summary:
    //    //     Removes the element at the specified index of the System.Collections.Generic.SortedList<TKey,TValue>.
    //    //
    //    // Parameters:
    //    //   index:
    //    //     The zero-based index of the element to remove.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentOutOfRangeException:
    //    //     index is less than zero.-or-index is equal to or greater than System.Collections.Generic.SortedList<TKey,TValue>.Count.
    //    public void RemoveAt(int index){}
    //    //
    //    // Summary:
    //    //     Sets the capacity to the actual number of elements in the System.Collections.Generic.SortedList<TKey,TValue>,
    //    //     if that number is less than 90 percent of current capacity.
    //    public void TrimExcess(){}
    //    //
    //    // Summary:
    //    //     Gets the value associated with the specified key.
    //    //
    //    // Parameters:
    //    //   key:
    //    //     The key whose value to get.
    //    //
    //    //   value:
    //    //     When this method returns, the value associated with the specified key, if
    //    //     the key is found; otherwise, the default value for the type of the value
    //    //     parameter. This parameter is passed uninitialized.
    //    //
    //    // Returns:
    //    //     true if the System.Collections.Generic.SortedList<TKey,TValue> contains an
    //    //     element with the specified key; otherwise, false.
    //    //
    //    // Exceptions:
    //    //   System.ArgumentNullException:
    //    //     key is null.
    //    public bool TryGetValue(TKey key, out TValue value)
    //    {
    //        value = default(TValue);
    //        return false;
    //    }

    //    #region Implementation of IEnumerable

    //    /// <summary>
    //    /// Returns an enumerator that iterates through a collection.
    //    /// </summary>
    //    /// <returns>
    //    /// An <see cref="T:System.Collections.IEnumerator"/> object that can be used to iterate through the collection.
    //    /// </returns>
    //    IEnumerator IEnumerable.GetEnumerator()
    //    {
    //        return GetEnumerator();
    //    }

    //    #endregion
    //}
}
