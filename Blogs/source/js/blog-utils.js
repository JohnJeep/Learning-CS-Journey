(function () {
  'use strict';

  // ── Swap post date display order ──────────────────────────────────────────
  // Butterfly v5 renders dates with CSS classes (not IDs):
  //   time.post-meta-date-created  (发表于) — appears first in DOM
  //   time.post-meta-date-updated  (更新于) — appears second in DOM
  // Each <time> is wrapped in a <span.post-meta-date> block; a
  // <span.post-meta-separator> sits between the two blocks.
  // We reorder to: updatedBlock | separator | createdBlock.
  function swapPostDates() {
    var createdTime = document.querySelector('time.post-meta-date-created');
    var updatedTime = document.querySelector('time.post-meta-date-updated');
    if (!createdTime || !updatedTime) return;

    var createdBlock = createdTime.closest('span.post-meta-date') || createdTime.parentNode;
    var updatedBlock = updatedTime.closest('span.post-meta-date') || updatedTime.parentNode;
    var parent = createdBlock.parentNode;
    if (!parent || createdBlock.parentNode !== updatedBlock.parentNode) return;

    // Find the separator node between the two date blocks
    var separator = null;
    var node = createdBlock.nextSibling;
    while (node && node !== updatedBlock) {
      if (node.nodeType === 1) { separator = node; break; }
      node = node.nextSibling;
    }

    // Move updatedBlock before createdBlock, keep separator in between
    parent.insertBefore(updatedBlock, createdBlock);
    if (separator) parent.insertBefore(separator, createdBlock);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', swapPostDates);
  } else {
    swapPostDates();
  }
})();
