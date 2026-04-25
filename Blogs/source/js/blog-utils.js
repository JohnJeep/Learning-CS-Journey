(function () {
  'use strict';

  // ── Swap post date display order ──────────────────────────────────────────
  // Config has date_type: both → Butterfly renders #post-date (发表于) first,
  // then #post-lastmod (更新于). We want updated first, created second.
  function swapPostDates() {
    var created = document.getElementById('post-date');
    var updated = document.getElementById('post-lastmod');
    if (created && updated && created.parentNode === updated.parentNode) {
      // insert updated *before* created — physically reorders in DOM
      created.parentNode.insertBefore(updated, created);
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', swapPostDates);
  } else {
    swapPostDates();
  }
})();
