using System;
using System.Collections;
using System.Collections.Generic;

namespace SL3Reader
{
    public class FrameList : List<int>,
                             IEnumerable<IFrame>, IEnumerable,
                             IReadOnlyList<IFrame>,
                             IReadOnlyCollection<IFrame>
    {
        private readonly IReadOnlyList<IFrame> contents;

        public FrameList(IReadOnlyList<IFrame> contents) : base() =>
            this.contents = contents ?? throw new ArgumentNullException(nameof(contents));

        // Need to remove redundant content.
        public FrameList(int capacity, IReadOnlyList<IFrame> contents) : base(capacity) =>
             this.contents = contents ?? throw new ArgumentNullException(nameof(contents));

        public new IFrame this[int index] => contents[base[index]]; // The new keyword hides the orignal "this".

        IEnumerator<IFrame> IEnumerable<IFrame>.GetEnumerator()
        {
            throw new NotImplementedException();
        }
    }
}