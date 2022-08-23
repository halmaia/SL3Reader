using System;
using System.Collections;
using System.Collections.Generic;

namespace SL3Reader
{
    public class FrameList : List<int>,
                             IEnumerable<Frame>, IEnumerable,
                             IReadOnlyList<Frame>,
                             IReadOnlyCollection<Frame>
    {
        private readonly IReadOnlyList<Frame> contents;

        public FrameList(IReadOnlyList<Frame> contents) : base() =>
            this.contents = contents ?? throw new ArgumentNullException(nameof(contents));

        // Need to remove redundant content.
        public FrameList(int capacity, IReadOnlyList<Frame> contents) : base(capacity) =>
             this.contents = contents ?? throw new ArgumentNullException(nameof(contents));

        public new Frame this[int index] => contents[index]; // The new keyword hides the orignal "this".
       
        IEnumerator<Frame> IEnumerable<Frame>.GetEnumerator() => 
            contents.GetEnumerator();
    }
}